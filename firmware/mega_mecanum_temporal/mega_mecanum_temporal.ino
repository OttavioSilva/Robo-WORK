// Variante temporal de workotvio, preservando protocolo/motores v2. NAO gravada automaticamente.
#include <Arduino.h>
#include "sliding_scan.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// Base mecanum + LiDAR Delta-2D + BNO055 para Arduino Mega 2560.
// Projeto sem encoders. O BNO055 e somente sensor nesta etapa:
// ele nao corrige os motores e nao interfere nas rotacoes pedidas pelo ROS 2.

namespace Config {

constexpr uint32_t USB_BAUD = 500000UL;
constexpr uint32_t LIDAR_BAUD = 115200UL;

constexpr uint8_t LIDAR_MOTOR_PWM_PIN = 44;
constexpr uint8_t LIDAR_START_PWM = 255;
constexpr uint8_t LIDAR_WORK_PWM = 200;
constexpr uint32_t LIDAR_START_TIME_MS = 2000UL;

constexpr int16_t MOTOR_MAX_PWM = 215;
// O computador define o PWM minimo adequado para cada movimento. Isso permite
// usar PWM 150 na translacao e um valor menor em uma rotacao intencional.
constexpr float MOTOR_ACCEL_PWM_PER_S = 200.0F;
constexpr float MOTOR_DECEL_PWM_PER_S = 600.0F;
constexpr uint32_t MOTOR_UPDATE_MS = 5UL;
constexpr uint32_t COMMAND_WATCHDOG_MS = 300UL;

constexpr uint16_t LIDAR_MIN_MM = 80;
constexpr uint16_t LIDAR_MAX_MM = 6000;
constexpr uint16_t SCAN_BIN_COUNT = 180;
constexpr float SCAN_BIN_DEG = 2.0F;
constexpr uint16_t MIN_VALID_SCAN_BINS = 25;
constexpr float SENSOR_FRONT_DEG = 270.0F;

constexpr uint32_t STATUS_PERIOD_MS = 200UL;

constexpr uint8_t BNO055_ADDRESS = 0x29;
// 25 Hz e suficiente para a primeira integracao com robot_localization e
// reduz o tempo bloqueado em transacoes I2C no Mega.
constexpr uint32_t IMU_PERIOD_MS = 40UL;
constexpr uint32_t IMU_CALIBRATION_PERIOD_MS = 500UL;

}  // namespace Config

struct MotorPins {
  uint8_t forward_pwm;
  uint8_t reverse_pwm;
  bool inverted;
};

// Mapeamento e inversoes confirmados no teste BNO055 de 17/09/2026.
// forward_pwm = RPWM; reverse_pwm = LPWM.
// A inversao faz PWM logico positivo sempre representar avanco fisico da roda.
constexpr MotorPins M1{12, 11, true};  // dianteira direita (FR)
constexpr MotorPins M2{10, 9, false};  // dianteira esquerda (FL)
constexpr MotorPins M3{8, 7, true};    // traseira direita (RR)
constexpr MotorPins M4{6, 5, false};   // traseira esquerda (RL)

namespace UsbProtocol {

constexpr uint8_t SOF_1 = 0xA5;
constexpr uint8_t SOF_2 = 0x5A;
// A versao 2 impede que o bridge diferencial antigo comande este firmware.
constexpr uint8_t VERSION = 0x02;

constexpr uint8_t TYPE_MECANUM_COMMAND = 0x01;
constexpr uint8_t TYPE_SCAN = 0x10;
constexpr uint8_t TYPE_STATUS = 0x11;
constexpr uint8_t TYPE_IMU = 0x12;
constexpr uint8_t TYPE_SCAN_TIMED = 0x13;
constexpr uint8_t TYPE_CLOCK_PROBE_REPLY = 0x14;
constexpr uint8_t TYPE_IMU_TIMED = 0x15;
constexpr uint8_t TYPE_LIDAR_PACKET = 0x16;
constexpr uint8_t TYPE_CLOCK_PROBE = 0x02;

constexpr uint8_t FLAG_ENABLE = 0x01;
constexpr uint8_t FLAG_ESTOP = 0x02;

// sequencia u16 + FL/FR/RL/RR i16 + flags u8
constexpr uint16_t MECANUM_COMMAND_PAYLOAD_SIZE = 11;
constexpr uint16_t MAX_RX_PAYLOAD_SIZE = 32;

}  // namespace UsbProtocol

namespace DeltaProtocol {

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t VERSION = 0x01;
constexpr uint8_t PACKET_TYPE = 0x61;
constexpr uint8_t DATA_RPM_AND_MEAS = 0xAD;
constexpr uint8_t DATA_RPM_ONLY = 0xAE;
constexpr uint16_t MIN_FRAME_SIZE = 11;
constexpr uint16_t MAX_FRAME_SIZE = 160;
constexpr uint16_t MAX_SAMPLE_COUNT =
    (MAX_FRAME_SIZE - 15U) / 3U;
constexpr uint8_t PACKETS_PER_SCAN = 16;
constexpr uint32_t INTERBYTE_TIMEOUT_MS = 50UL;

}  // namespace DeltaProtocol

// ---------- Utilitarios binarios ----------

uint16_t readU16LE(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int16_t readI16LE(const uint8_t *data) {
  return static_cast<int16_t>(readU16LE(data));
}

uint16_t readU16BE(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) |
         static_cast<uint16_t>(data[1]);
}

uint16_t crc16CcittUpdate(uint16_t crc, uint8_t value) {
  crc ^= static_cast<uint16_t>(value) << 8;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                          : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

// ---------- Fila USB TX nao bloqueante ----------

constexpr uint16_t USB_TX_BUFFER_SIZE = 1024;
uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
uint16_t usb_tx_head = 0;
uint16_t usb_tx_tail = 0;
uint32_t usb_tx_dropped_frames = 0;

uint16_t usbTxUsed() {
  if (usb_tx_head >= usb_tx_tail) {
    return usb_tx_head - usb_tx_tail;
  }
  return USB_TX_BUFFER_SIZE - (usb_tx_tail - usb_tx_head);
}

uint16_t usbTxFree() {
  return USB_TX_BUFFER_SIZE - usbTxUsed() - 1;
}

void usbTxPush(uint8_t value) {
  usb_tx_buffer[usb_tx_head] = value;
  usb_tx_head = (usb_tx_head + 1U) % USB_TX_BUFFER_SIZE;
}

void usbFramePush(uint8_t value, uint16_t &crc) {
  usbTxPush(value);
  crc = crc16CcittUpdate(crc, value);
}

void usbFramePushU16(uint16_t value, uint16_t &crc) {
  usbFramePush(static_cast<uint8_t>(value & 0xFFU), crc);
  usbFramePush(static_cast<uint8_t>((value >> 8) & 0xFFU), crc);
}

void usbFramePushI16(int16_t value, uint16_t &crc) {
  usbFramePushU16(static_cast<uint16_t>(value), crc);
}

void usbFramePushU32(uint32_t value, uint16_t &crc) {
  usbFramePush(static_cast<uint8_t>(value & 0xFFUL), crc);
  usbFramePush(static_cast<uint8_t>((value >> 8) & 0xFFUL), crc);
  usbFramePush(static_cast<uint8_t>((value >> 16) & 0xFFUL), crc);
  usbFramePush(static_cast<uint8_t>((value >> 24) & 0xFFUL), crc);
}

void usbFramePushI32(int32_t value, uint16_t &crc) {
  usbFramePushU32(static_cast<uint32_t>(value), crc);
}

bool usbFrameBegin(uint8_t type, uint16_t payload_size, uint16_t &crc) {
  const uint16_t total_size = 2U + 4U + payload_size + 2U;
  if (usbTxFree() < total_size) {
    ++usb_tx_dropped_frames;
    return false;
  }

  usbTxPush(UsbProtocol::SOF_1);
  usbTxPush(UsbProtocol::SOF_2);

  crc = 0xFFFFU;
  usbFramePush(UsbProtocol::VERSION, crc);
  usbFramePush(type, crc);
  usbFramePushU16(payload_size, crc);
  return true;
}

void usbFrameEnd(uint16_t crc) {
  usbTxPush(static_cast<uint8_t>(crc & 0xFFU));
  usbTxPush(static_cast<uint8_t>((crc >> 8) & 0xFFU));
}

void serviceUsbTx() {
  int writable = Serial.availableForWrite();
  while (writable-- > 0 && usb_tx_tail != usb_tx_head) {
    Serial.write(usb_tx_buffer[usb_tx_tail]);
    usb_tx_tail = (usb_tx_tail + 1U) % USB_TX_BUFFER_SIZE;
  }
}

// ---------- Controle dos motores ----------

enum WheelIndex : uint8_t {
  WHEEL_FL = 0,
  WHEEL_FR = 1,
  WHEEL_RL = 2,
  WHEEL_RR = 3,
  WHEEL_COUNT = 4
};

int16_t target_wheel_pwm[WHEEL_COUNT] = {0, 0, 0, 0};
float current_wheel_pwm[WHEEL_COUNT] = {0.0F, 0.0F, 0.0F, 0.0F};
bool command_received = false;
bool watchdog_active = true;
uint32_t last_valid_command_ms = 0;
uint32_t last_motor_update_ms = 0;
uint16_t last_command_sequence = 0;

int16_t sanitizeRequestedPwm(int16_t value) {
  if (value == 0) {
    return 0;
  }

  int32_t magnitude = value;
  const int16_t sign = magnitude < 0 ? -1 : 1;
  if (magnitude < 0) {
    magnitude = -magnitude;
  }
  if (magnitude > Config::MOTOR_MAX_PWM) {
    magnitude = Config::MOTOR_MAX_PWM;
  }
  return static_cast<int16_t>(sign * magnitude);
}

void writeSingleMotor(const MotorPins &pins, int16_t signed_pwm) {
  int16_t value = signed_pwm;
  if (pins.inverted) {
    value = -value;
  }

  const uint8_t magnitude = static_cast<uint8_t>(abs(value));
  if (value > 0) {
    analogWrite(pins.reverse_pwm, 0);
    analogWrite(pins.forward_pwm, magnitude);
  } else if (value < 0) {
    analogWrite(pins.forward_pwm, 0);
    analogWrite(pins.reverse_pwm, magnitude);
  } else {
    analogWrite(pins.forward_pwm, 0);
    analogWrite(pins.reverse_pwm, 0);
  }
}

void writeWheel(uint8_t wheel, int16_t pwm) {
  switch (wheel) {
    case WHEEL_FL:
      writeSingleMotor(M2, pwm);
      break;
    case WHEEL_FR:
      writeSingleMotor(M1, pwm);
      break;
    case WHEEL_RL:
      writeSingleMotor(M4, pwm);
      break;
    case WHEEL_RR:
      writeSingleMotor(M3, pwm);
      break;
  }
}

void stopDriveImmediately() {
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    target_wheel_pwm[wheel] = 0;
    current_wheel_pwm[wheel] = 0.0F;
    writeWheel(wheel, 0);
  }
}

float stepMotorRamp(float current, int16_t requested, float elapsed_s) {
  // Um comando zero significa parada. O corte imediato evita que a rampa de
  // desaceleracao continue impulsionando o robo depois de atingir o alvo.
  if (requested == 0) {
    return 0.0F;
  }

  if (fabsf(current) < 0.5F) {
    current = 0.0F;
  }

  // O bridge envia o PWM minimo apropriado para translacao ou rotacao.
  // Todas as rodas partem juntas diretamente no valor solicitado.
  if (current == 0.0F) {
    return static_cast<float>(requested);
  }

  float effective_target = static_cast<float>(requested);
  const bool reversing = (current > 0.0F && effective_target < 0.0F) ||
                         (current < 0.0F && effective_target > 0.0F);
  if (reversing) {
    effective_target = 0.0F;
  }

  const bool decelerating = fabsf(effective_target) < fabsf(current);
  const float rate = decelerating ? Config::MOTOR_DECEL_PWM_PER_S
                                  : Config::MOTOR_ACCEL_PWM_PER_S;
  const float maximum_step = rate * elapsed_s;

  if (current < effective_target) {
    current += maximum_step;
    if (current > effective_target) {
      current = effective_target;
    }
  } else if (current > effective_target) {
    current -= maximum_step;
    if (current < effective_target) {
      current = effective_target;
    }
  }

  if (fabsf(current) < 0.5F && effective_target == 0.0F) {
    current = 0.0F;
  }
  return current;
}

void serviceMotors(uint32_t now_ms) {
  if (command_received &&
      static_cast<uint32_t>(now_ms - last_valid_command_ms) > Config::COMMAND_WATCHDOG_MS) {
    watchdog_active = true;
    command_received = false;
    stopDriveImmediately();
  }

  const uint32_t elapsed_ms = now_ms - last_motor_update_ms;
  if (elapsed_ms < Config::MOTOR_UPDATE_MS) {
    return;
  }
  last_motor_update_ms = now_ms;

  if (watchdog_active) {
    stopDriveImmediately();
    return;
  }

  const float elapsed_s = static_cast<float>(elapsed_ms) * 0.001F;
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    current_wheel_pwm[wheel] =
        stepMotorRamp(current_wheel_pwm[wheel], target_wheel_pwm[wheel], elapsed_s);
    writeWheel(wheel, static_cast<int16_t>(lroundf(current_wheel_pwm[wheel])));
  }
}

// ---------- Parser de comandos USB ----------

uint8_t usb_rx_buffer[4U + UsbProtocol::MAX_RX_PAYLOAD_SIZE + 2U];
uint16_t usb_rx_index = 0;
uint16_t usb_rx_expected = 0;
bool usb_rx_receiving = false;
bool usb_rx_saw_first_sof = false;
uint32_t usb_rx_bad_frames = 0;

void resetUsbRx() {
  usb_rx_index = 0;
  usb_rx_expected = 0;
  usb_rx_receiving = false;
}

void applyMecanumCommand(const uint8_t *payload, uint32_t now_ms) {
  const uint16_t sequence = readU16LE(payload);
  const uint8_t flags = payload[10];

  last_command_sequence = sequence;
  last_valid_command_ms = now_ms;
  command_received = true;

  if ((flags & UsbProtocol::FLAG_ENABLE) == 0U ||
      (flags & UsbProtocol::FLAG_ESTOP) != 0U) {
    watchdog_active = true;
    command_received = false;
    stopDriveImmediately();
    return;
  }

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    target_wheel_pwm[wheel] = sanitizeRequestedPwm(
        readI16LE(payload + 2U + static_cast<uint16_t>(wheel) * 2U));
  }
  watchdog_active = false;
}

uint32_t temporal_host_seen_ms = 0;
bool temporalEnabled() {
  return temporal_host_seen_ms && static_cast<uint32_t>(millis()-temporal_host_seen_ms) < 2000UL;
}

void processUsbRxFrame(uint32_t now_ms) {
  const uint16_t payload_size = readU16LE(usb_rx_buffer + 2);
  const uint16_t crc_offset = 4U + payload_size;
  const uint16_t received_crc = readU16LE(usb_rx_buffer + crc_offset);

  uint16_t calculated_crc = 0xFFFFU;
  for (uint16_t i = 0; i < crc_offset; ++i) {
    calculated_crc = crc16CcittUpdate(calculated_crc, usb_rx_buffer[i]);
  }

  if (received_crc != calculated_crc || usb_rx_buffer[0] != UsbProtocol::VERSION) {
    ++usb_rx_bad_frames;
    return;
  }

  if (usb_rx_buffer[1] == UsbProtocol::TYPE_CLOCK_PROBE && payload_size == 4U) {
    temporal_host_seen_ms = millis();
    const uint32_t observed_us = micros();
    uint16_t crc = 0;
    if (usbFrameBegin(UsbProtocol::TYPE_CLOCK_PROBE_REPLY, 8U, crc)) {
      usbFramePushU32(readU16LE(usb_rx_buffer + 4) |
        (static_cast<uint32_t>(readU16LE(usb_rx_buffer + 6)) << 16), crc);
      usbFramePushU32(observed_us, crc);
      usbFrameEnd(crc);
    }
    return;
  }
  if (usb_rx_buffer[1] == UsbProtocol::TYPE_MECANUM_COMMAND &&
      payload_size == UsbProtocol::MECANUM_COMMAND_PAYLOAD_SIZE) {
    applyMecanumCommand(usb_rx_buffer + 4, now_ms);
  } else {
    ++usb_rx_bad_frames;
  }
}

void processUsbRxByte(uint8_t value, uint32_t now_ms) {
  if (!usb_rx_receiving) {
    if (!usb_rx_saw_first_sof) {
      usb_rx_saw_first_sof = value == UsbProtocol::SOF_1;
      return;
    }

    if (value == UsbProtocol::SOF_2) {
      usb_rx_receiving = true;
      usb_rx_saw_first_sof = false;
      usb_rx_index = 0;
      usb_rx_expected = 0;
      return;
    }

    usb_rx_saw_first_sof = value == UsbProtocol::SOF_1;
    return;
  }

  if (usb_rx_index >= sizeof(usb_rx_buffer)) {
    ++usb_rx_bad_frames;
    resetUsbRx();
    return;
  }

  usb_rx_buffer[usb_rx_index++] = value;

  if (usb_rx_index == 4U) {
    const uint16_t payload_size = readU16LE(usb_rx_buffer + 2);
    if (payload_size > UsbProtocol::MAX_RX_PAYLOAD_SIZE) {
      ++usb_rx_bad_frames;
      resetUsbRx();
      return;
    }
    usb_rx_expected = 4U + payload_size + 2U;
  }

  if (usb_rx_expected != 0U && usb_rx_index == usb_rx_expected) {
    processUsbRxFrame(now_ms);
    resetUsbRx();
  }
}

void serviceUsbRx(uint32_t now_ms) {
  while (Serial.available() > 0) {
    processUsbRxByte(static_cast<uint8_t>(Serial.read()), now_ms);
  }
}

// ---------- Delta-2D e montagem da varredura ----------

uint8_t lidar_frame[DeltaProtocol::MAX_FRAME_SIZE];
uint16_t lidar_frame_index = 0;
uint16_t lidar_expected_size = 0;
uint32_t lidar_last_byte_ms = 0;

SlidingScan180 temporal_scan;
uint32_t temporal_last_snapshot_us = 0;
uint8_t temporal_samples_min = 255, temporal_samples_max = 0, temporal_packet_count = 0;
uint16_t scan_ranges_mm[Config::SCAN_BIN_COUNT];
uint16_t scan_valid_bins = 0;
uint16_t last_scan_valid_bins = 0;
uint16_t scan_sequence = 0;
uint32_t scan_started_us = 0;
bool scan_active = false;
float previous_packet_start_deg = 0.0F;
bool have_previous_packet_angle = false;
uint8_t lidar_scan_freq_x20 = 0;

uint32_t lidar_good_packets = 0;
uint32_t lidar_bad_checksums = 0;
uint32_t lidar_bad_frames = 0;

uint32_t lidar_bad_timeout = 0;
uint32_t lidar_bad_size = 0;
uint32_t lidar_bad_version = 0;
uint32_t lidar_bad_packet_type = 0;
uint32_t lidar_bad_measurement = 0;
uint32_t lidar_bad_unknown_type = 0;
uint32_t lidar_bad_overflow = 0;

uint16_t lidar_max_serial2_available = 0;
uint32_t max_loop_gap_us = 0;
uint32_t last_loop_entry_us = 0;

// Supervisao / auto-recuperacao do Delta-2D.
uint32_t lidar_rx_bytes_total = 0;
uint32_t lidar_last_rx_byte_ms = 0;
uint32_t lidar_last_good_packet_ms = 0;
uint32_t lidar_last_completed_scan_ms = 0;
uint32_t lidar_last_recovery_ms = 0;

uint32_t lidar_parser_recoveries = 0;
uint32_t lidar_scan_recoveries = 0;
uint32_t lidar_uart_recoveries = 0;

bool lidar_seen_data = false;

// Normalmente temos ~5.7 scans/s (~175 ms por volta).
// Estes limites deixam margem ampla, mas tentam recuperar antes do
// source_timeout=1.0 s usado pelo Collision Monitor.
constexpr uint32_t LIDAR_PACKET_STALL_MS = 500UL;
constexpr uint32_t LIDAR_SCAN_STALL_MS = 650UL;
constexpr uint32_t LIDAR_UART_SILENCE_MS = 800UL;
constexpr uint32_t LIDAR_RECOVERY_COOLDOWN_MS = 500UL;

void clearScan() {
  temporal_samples_min = 255;
  temporal_samples_max = temporal_packet_count = 0;
  for (uint16_t i = 0; i < Config::SCAN_BIN_COUNT; ++i) {
    scan_ranges_mm[i] = 0;
  }
  scan_valid_bins = 0;
}

float normalizeAngle180(float angle_deg) {
  while (angle_deg >= 180.0F) {
    angle_deg -= 360.0F;
  }
  while (angle_deg < -180.0F) {
    angle_deg += 360.0F;
  }
  return angle_deg;
}

void queueCompletedScan(uint32_t scan_time_us) {
  last_scan_valid_bins = scan_valid_bins;
  if (scan_valid_bins < Config::MIN_VALID_SCAN_BINS) {
    return;
  }

  // O LiDAR conseguiu construir uma varredura completa e valida.
  // Marcamos antes da fila USB para separar saude do sensor de eventual
  // congestionamento na transmissao USB.
  lidar_last_completed_scan_ms = millis();

  constexpr uint16_t payload_size =
      2U + 4U + 4U + 1U + 2U + (Config::SCAN_BIN_COUNT * 2U);
  uint16_t crc = 0;
  if (!usbFrameBegin(UsbProtocol::TYPE_SCAN, payload_size, crc)) {
    return;
  }

  usbFramePushU16(scan_sequence++, crc);
  usbFramePushU32(millis(), crc);
  usbFramePushU32(scan_time_us, crc);
  usbFramePush(lidar_scan_freq_x20, crc);
  usbFramePushU16(scan_valid_bins, crc);
  for (uint16_t i = 0; i < Config::SCAN_BIN_COUNT; ++i) {
    usbFramePushU16(scan_ranges_mm[i], crc);
  }
  usbFrameEnd(crc);
}

// Envia uma janela completa de bins recentes a 40 Hz, independente da borda
// da revolucao (~5,7 Hz). Cada bin conserva a sua captura; velho/sem eco e zero.
// O quadro legado por revolucao continua inalterado para o caminho lateral.
void queueTemporalSnapshot() {
  const uint32_t now_us = micros();
  if (static_cast<uint32_t>(now_us-temporal_last_snapshot_us) < 25000UL) return;
  temporal_last_snapshot_us = now_us;
  temporal_scan.expire(now_us);  // impede ressuscitar dado velho apos wrap de micros
  if (!temporalEnabled()) return;
  const uint32_t oldest_us = temporal_scan.oldest(now_us);
  uint16_t valid = 0;
  for (uint16_t i=0; i<Config::SCAN_BIN_COUNT; ++i) if (temporal_scan.value(i,now_us)) ++valid;
  if (valid == 0) return;  // sem captura real, nao inventar timestamp novo
  uint16_t crc = 0;
  if (!usbFrameBegin(UsbProtocol::TYPE_SCAN_TIMED, 389U, crc)) return;
  usbFramePushU16(scan_sequence, crc);
  usbFramePushU32(millis(), crc);
  usbFramePushU32(now_us-oldest_us, crc);
  usbFramePush(lidar_scan_freq_x20, crc);
  usbFramePushU16(valid, crc);
  for (uint16_t i=0; i<Config::SCAN_BIN_COUNT; ++i)
    usbFramePushU16(temporal_scan.value(i,now_us), crc);
  usbFramePushU32(oldest_us, crc);
  usbFramePushU32(now_us, crc);
  usbFramePushU32(micros(), crc);
  usbFramePushU32(2UL | (static_cast<uint32_t>(temporal_samples_min) << 8) |
      (static_cast<uint32_t>(temporal_samples_max) << 16) |
      (static_cast<uint32_t>(temporal_packet_count) << 24), crc);
  usbFrameEnd(crc);
}

void resetLidarParser() {
  lidar_frame_index = 0;
  lidar_expected_size = 0;
}

bool validateDeltaChecksum(uint16_t frame_size) {
  uint16_t sum = 0;
  for (uint16_t i = 0; i < frame_size - 2U; ++i) {
    sum = static_cast<uint16_t>(sum + lidar_frame[i]);
  }
  return sum == readU16BE(lidar_frame + frame_size - 2U);
}

bool processDeltaMeasurement(uint16_t frame_size, uint32_t now_us) {
  const uint16_t packet_length = readU16BE(lidar_frame + 1);
  const uint16_t data_length = readU16BE(lidar_frame + 6);

  if (packet_length + 2U != frame_size ||
      packet_length != data_length + 8U ||
      data_length < 8U ||
      ((data_length - 5U) % 3U) != 0U) {
    ++lidar_bad_frames;
    ++lidar_bad_measurement;
    return false;
  }

  const uint16_t sample_count = (data_length - 5U) / 3U;

  if (sample_count == 0U ||
      sample_count > DeltaProtocol::MAX_SAMPLE_COUNT ||
      13U + sample_count * 3U + 2U != frame_size) {
    ++lidar_bad_frames;
    ++lidar_bad_measurement;
    return false;
  }

  lidar_scan_freq_x20 = lidar_frame[8];

  const float start_angle_deg =
      static_cast<float>(readU16BE(lidar_frame + 11)) * 0.01F;

  const bool completed_rotation =
      have_previous_packet_angle &&
      previous_packet_start_deg >= 300.0F &&
      start_angle_deg < 60.0F;

  if (completed_rotation && scan_active) {
    queueCompletedScan(now_us - scan_started_us);
    clearScan();
    scan_started_us = now_us;
  }

  if (!scan_active) {
    scan_active = true;
    scan_started_us = now_us;
    clearScan();
  }

  previous_packet_start_deg = start_angle_deg;
  have_previous_packet_angle = true;

  if (sample_count < temporal_samples_min) temporal_samples_min = sample_count;
  if (sample_count > temporal_samples_max) temporal_samples_max = sample_count;
  if (temporal_packet_count < 255) ++temporal_packet_count;
  const float angular_step_deg =
      360.0F /
      (static_cast<float>(DeltaProtocol::PACKETS_PER_SCAN) *
       sample_count);

  for (uint16_t i = 0; i < sample_count; ++i) {
    const uint16_t sample_offset = 13U + i * 3U;

    const uint16_t distance_mm_x4 =
        readU16BE(lidar_frame + sample_offset + 1U);

    const uint16_t distance_mm =
        static_cast<uint16_t>(distance_mm_x4 / 4U);

    float sensor_angle_deg =
        start_angle_deg + static_cast<float>(i) * angular_step_deg;

    while (sensor_angle_deg >= 360.0F) {
      sensor_angle_deg -= 360.0F;
    }

    // Delta-2D invertido:
    // 270 graus do sensor corresponde a frente do robo.
    const float robot_angle_deg =
        normalizeAngle180(
            Config::SENSOR_FRONT_DEG - sensor_angle_deg);

    int16_t bin = static_cast<int16_t>(
        floorf(
            (robot_angle_deg + 180.0F) /
            Config::SCAN_BIN_DEG));

    if (bin < 0) {
      bin = 0;
    } else if (
        bin >= static_cast<int16_t>(Config::SCAN_BIN_COUNT)) {
      bin = Config::SCAN_BIN_COUNT - 1;
    }

    const bool valid_distance = distance_mm >= Config::LIDAR_MIN_MM && distance_mm <= Config::LIDAR_MAX_MM;
    temporal_scan.sample(static_cast<uint16_t>(bin), valid_distance ? distance_mm : 0, now_us);
    if (!valid_distance) continue;

    uint16_t &stored_mm = scan_ranges_mm[bin];

    if (stored_mm == 0U) {
      stored_mm = distance_mm;
      ++scan_valid_bins;
    } else if (distance_mm < stored_mm) {
      stored_mm = distance_mm;
    }
  }

  return true;
}

void processCompleteDeltaFrame(uint16_t frame_size, uint32_t now_us) {
  // Preserva o pacote nativo para investigar resolucao, offset e qualidade.
  // Somente com host temporal ativo; nao altera o parser nem os bins legados.
  if (temporalEnabled()) {
    uint16_t raw_crc = 0;
    if (usbFrameBegin(UsbProtocol::TYPE_LIDAR_PACKET, frame_size+4U, raw_crc)) {
      usbFramePushU32(now_us, raw_crc);
      for (uint16_t i=0; i<frame_size; ++i) usbFramePush(lidar_frame[i], raw_crc);
      usbFrameEnd(raw_crc);
    }
  }
  if (!validateDeltaChecksum(frame_size)) {
    ++lidar_bad_checksums;
    return;
  }

  if (lidar_frame[3] != DeltaProtocol::VERSION) {
    ++lidar_bad_frames;
    ++lidar_bad_version;
    return;
  }

  if (lidar_frame[4] != DeltaProtocol::PACKET_TYPE) {
    ++lidar_bad_frames;
    ++lidar_bad_packet_type;
    return;
  }

  if (lidar_frame[5] == DeltaProtocol::DATA_RPM_AND_MEAS) {
    if (!processDeltaMeasurement(frame_size, now_us)) {
      return;
    }
  } else if (lidar_frame[5] == DeltaProtocol::DATA_RPM_ONLY) {
    if (frame_size >= DeltaProtocol::MIN_FRAME_SIZE) {
      lidar_scan_freq_x20 = lidar_frame[8];
    }
  } else {
    ++lidar_bad_frames;
    ++lidar_bad_unknown_type;
    return;
  }

  // Agora "good" significa realmente aceito pelo parser.
  ++lidar_good_packets;
  lidar_last_good_packet_ms = millis();
}

void processLidarByte(uint8_t value, uint32_t now_ms, uint32_t now_us) {
  if (lidar_frame_index != 0U &&
      static_cast<uint32_t>(now_ms - lidar_last_byte_ms) >
          DeltaProtocol::INTERBYTE_TIMEOUT_MS) {
    ++lidar_bad_frames;
    ++lidar_bad_timeout;
    resetLidarParser();
  }
  lidar_last_byte_ms = now_ms;

  if (lidar_frame_index == 0U && value != DeltaProtocol::START_BYTE) {
    return;
  }

  if (lidar_frame_index >= DeltaProtocol::MAX_FRAME_SIZE) {
    ++lidar_bad_frames;
    ++lidar_bad_overflow;
    resetLidarParser();
    return;
  }

  lidar_frame[lidar_frame_index++] = value;

  if (lidar_frame_index == 3U) {
    const uint16_t packet_length = readU16BE(lidar_frame + 1);
    lidar_expected_size = packet_length + 2U;
    if (lidar_expected_size < DeltaProtocol::MIN_FRAME_SIZE ||
        lidar_expected_size > DeltaProtocol::MAX_FRAME_SIZE) {
      ++lidar_bad_frames;
      ++lidar_bad_size;
      resetLidarParser();
      return;
    }
  }

  if (lidar_frame_index == 4U && lidar_frame[3] != DeltaProtocol::VERSION) {
    ++lidar_bad_frames;
    ++lidar_bad_version;
    resetLidarParser();
    return;
  }

  if (lidar_frame_index == 5U && lidar_frame[4] != DeltaProtocol::PACKET_TYPE) {
    ++lidar_bad_frames;
    ++lidar_bad_packet_type;
    resetLidarParser();
    return;
  }

  if (lidar_expected_size != 0U && lidar_frame_index == lidar_expected_size) {
    processCompleteDeltaFrame(lidar_expected_size, now_us);
    resetLidarParser();
  }
}

void serviceLidarRx() {
  const int available_now = Serial2.available();

  if (available_now > 0) {
    lidar_seen_data = true;

    if (static_cast<uint16_t>(available_now) >
        lidar_max_serial2_available) {
      lidar_max_serial2_available =
          static_cast<uint16_t>(available_now);
    }
  }

  uint16_t processed = 0;

  // Antes: 96 bytes.
  // O RX de 256 bytes ja chegou a 255 em testes reais.
  // 192 reduz fortemente o backlog sem deixar esta rotina monopolizar o loop.
  while (Serial2.available() > 0 && processed < 192U) {
    const uint32_t now_ms = millis();
    const uint32_t now_us = micros();

    const int raw = Serial2.read();

    if (raw < 0) {
      break;
    }

    ++lidar_rx_bytes_total;
    lidar_last_rx_byte_ms = now_ms;

    processLidarByte(
        static_cast<uint8_t>(raw),
        now_ms,
        now_us);

    ++processed;
  }
}


void resetLidarScanAssembly() {
  clearScan();

  scan_active = false;
  scan_started_us = 0;

  previous_packet_start_deg = 0.0F;
  have_previous_packet_angle = false;
}

void serviceLidarHealth(uint32_t now_ms) {
  // Nao fazemos nada antes de o Delta-2D ter enviado pelo menos um byte.
  if (!lidar_seen_data) {
    return;
  }

  const uint32_t byte_age =
      static_cast<uint32_t>(
          now_ms - lidar_last_rx_byte_ms);

  const uint32_t packet_age =
      lidar_last_good_packet_ms == 0U
          ? 0U
          : static_cast<uint32_t>(
                now_ms - lidar_last_good_packet_ms);

  const uint32_t scan_age =
      lidar_last_completed_scan_ms == 0U
          ? 0U
          : static_cast<uint32_t>(
                now_ms - lidar_last_completed_scan_ms);

  const uint32_t recovery_age =
      static_cast<uint32_t>(
          now_ms - lidar_last_recovery_ms);

  // --------------------------------------------------------
  // CASO A:
  // Bytes continuam chegando, mas nenhum pacote valido fecha.
  // Isso indica parser fora de fase / fragmento corrompido.
  // --------------------------------------------------------
  if (lidar_last_good_packet_ms != 0U &&
      byte_age < 150UL &&
      packet_age > LIDAR_PACKET_STALL_MS &&
      recovery_age > LIDAR_RECOVERY_COOLDOWN_MS) {

    resetLidarParser();
    resetLidarScanAssembly();

    ++lidar_parser_recoveries;
    lidar_last_recovery_ms = now_ms;

    return;
  }

  // --------------------------------------------------------
  // CASO B:
  // Pacotes validos continuam chegando, mas nao conseguimos
  // concluir uma rotacao / scan.
  // --------------------------------------------------------
  if (lidar_last_completed_scan_ms != 0U &&
      lidar_last_good_packet_ms != 0U &&
      packet_age < 200UL &&
      scan_age > LIDAR_SCAN_STALL_MS &&
      recovery_age > LIDAR_RECOVERY_COOLDOWN_MS) {

    resetLidarScanAssembly();

    ++lidar_scan_recoveries;
    lidar_last_completed_scan_ms = now_ms;
    lidar_last_recovery_ms = now_ms;

    return;
  }

  // --------------------------------------------------------
  // CASO C:
  // Nenhum byte na UART por tempo anormal.
  // Reinicia somente o periferico Serial2, sem reiniciar Mega.
  // --------------------------------------------------------
  if (byte_age > LIDAR_UART_SILENCE_MS &&
      recovery_age > LIDAR_RECOVERY_COOLDOWN_MS) {

    Serial2.end();
    Serial2.begin(Config::LIDAR_BAUD);

    resetLidarParser();
    resetLidarScanAssembly();

    ++lidar_uart_recoveries;
    lidar_last_recovery_ms = now_ms;

    // Impede novo restart a cada iteracao enquanto aguardamos
    // a retomada do fluxo.
    lidar_last_rx_byte_ms = now_ms;

    return;
  }
}

// ---------- BNO055: somente sensor ----------

Adafruit_BNO055 bno = Adafruit_BNO055(55, Config::BNO055_ADDRESS);
bool imu_present = false;
uint16_t imu_sequence = 0;
uint32_t last_imu_ms = 0;
uint32_t last_imu_calibration_ms = 0;
uint8_t imu_cal_system = 0;
uint8_t imu_cal_gyro = 0;
uint8_t imu_cal_accel = 0;
uint8_t imu_cal_mag = 0;

// Robustez BNO055.
// imu_present passa a significar: sensor disponivel E confiavel para navegacao.
uint32_t last_imu_health_check_ms = 0;
uint32_t last_imu_recovery_ms = 0;
uint32_t last_good_imu_ms = 0;
uint32_t imu_fault_count = 0;

bool imu_have_heading = false;
bool imu_rebase_pending = false;

float imu_last_raw_heading_deg = 0.0F;
float imu_continuous_heading_deg = 0.0F;

constexpr uint32_t IMU_HEALTH_CHECK_MS = 500UL;
constexpr uint32_t IMU_RECOVERY_PERIOD_MS = 500UL;

// O robo esta limitado a velocidades angulares muito menores que isto.
// Serve para rejeitar saltos impossiveis sem confundir passagem 359 -> 0.
constexpr float IMU_MAX_PLAUSIBLE_RATE_DEG_S = 60.0F;
constexpr float IMU_MIN_ALLOWED_JUMP_DEG = 3.0F;

// Registradores publicos do BNO055.
constexpr uint8_t BNO055_CHIP_ID_REG = 0x00;
constexpr uint8_t BNO055_EXPECTED_CHIP_ID = 0xA0;
constexpr uint8_t BNO055_OPR_MODE_REG = 0x3D;
constexpr uint8_t BNO055_NDOF_MODE = 0x0C;

int16_t saturateI16(long value) {
  if (value > 32767L) return 32767;
  if (value < -32768L) return -32768;
  return static_cast<int16_t>(value);
}

float normalizeHeading360(float angle_deg) {
  while (angle_deg >= 360.0F) {
    angle_deg -= 360.0F;
  }
  while (angle_deg < 0.0F) {
    angle_deg += 360.0F;
  }
  return angle_deg;
}

float shortestHeadingDelta(float current_deg, float previous_deg) {
  float delta = current_deg - previous_deg;

  while (delta > 180.0F) {
    delta -= 360.0F;
  }
  while (delta < -180.0F) {
    delta += 360.0F;
  }

  return delta;
}

bool readBnoRegisterSafe(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(Config::BNO055_ADDRESS);
  Wire.write(reg);

  const uint8_t tx_result = Wire.endTransmission(false);
  if (tx_result != 0U) {
    return false;
  }

  const uint8_t received =
      Wire.requestFrom(Config::BNO055_ADDRESS, static_cast<uint8_t>(1U));

  if (received != 1U || Wire.available() <= 0) {
    return false;
  }

  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool bnoRuntimeHealthy() {
  uint8_t chip_id = 0;
  uint8_t mode = 0;

  if (!readBnoRegisterSafe(BNO055_CHIP_ID_REG, chip_id) ||
      chip_id != BNO055_EXPECTED_CHIP_ID) {
    return false;
  }

  // Nao deixa a checagem do BNO acumular bytes do Delta-2D.
  serviceLidarRx();
  serviceUsbRx(millis());
  serviceUsbTx();

  if (!readBnoRegisterSafe(BNO055_OPR_MODE_REG, mode)) {
    return false;
  }

  serviceLidarRx();
  serviceUsbRx(millis());

  return mode == BNO055_NDOF_MODE;
}

void markImuFault() {
  if (imu_present) {
    ++imu_fault_count;
  }

  imu_present = false;

  // Mantemos imu_continuous_heading_deg congelado.
  // Quando o BNO voltar, a primeira leitura apenas reancora o valor bruto,
  // sem produzir salto na orientacao entregue ao ROS.
  imu_rebase_pending = imu_have_heading;

  stopDriveImmediately();
}

void serviceImuRecovery(uint32_t now_ms) {
  if (imu_present ||
      static_cast<uint32_t>(now_ms - last_imu_recovery_ms) <
          IMU_RECOVERY_PERIOD_MS) {
    return;
  }

  last_imu_recovery_ms = now_ms;

  uint8_t chip_id = 0;
  if (!readBnoRegisterSafe(BNO055_CHIP_ID_REG, chip_id) ||
      chip_id != BNO055_EXPECTED_CHIP_ID) {
    return;
  }

  serviceLidarRx();
  serviceUsbRx(millis());
  serviceUsbTx();

  uint8_t mode = 0;
  if (!readBnoRegisterSafe(BNO055_OPR_MODE_REG, mode)) {
    return;
  }

  if (mode != BNO055_NDOF_MODE) {
    // Recuperacao leve. Evitamos bno.begin() em runtime porque a rotina da
    // biblioteca pode bloquear por bastante tempo durante uma reinicializacao.
    serviceLidarRx();
    serviceUsbTx();

    bno.setMode(OPERATION_MODE_NDOF);

    // setMode() possui um pequeno delay. Descartamos qualquer fragmento de
    // pacote acumulado nesse intervalo para o Delta-2D resincronizar limpo.
    while (Serial2.available() > 0) {
      Serial2.read();
    }
    resetLidarParser();

    if (!readBnoRegisterSafe(BNO055_OPR_MODE_REG, mode) ||
        mode != BNO055_NDOF_MODE) {
      return;
    }
  }

  imu_present = true;
  imu_rebase_pending = imu_have_heading;
  last_imu_ms = now_ms;
  last_imu_health_check_ms = now_ms;
}

void serviceImu(uint32_t now_ms) {
  if (!imu_present) {
    serviceImuRecovery(now_ms);
    return;
  }

  if (static_cast<uint32_t>(now_ms - last_imu_ms) <
      Config::IMU_PERIOD_MS) {
    return;
  }

  last_imu_ms = now_ms;

  // Checagem direta e barata do CHIP_ID + modo NDOF.
  // Nao usamos getSystemStatus() aqui porque a biblioteca introduz delay
  // significativo nessa chamada.
  if (static_cast<uint32_t>(now_ms - last_imu_health_check_ms) >=
      IMU_HEALTH_CHECK_MS) {
    last_imu_health_check_ms = now_ms;

    if (!bnoRuntimeHealthy()) {
      markImuFault();
      return;
    }
  }

  const uint32_t imu_capture_begin_us = micros();
  sensors_event_t orientation_event;
  sensors_event_t gyro_event;

  bno.getEvent(&orientation_event, Adafruit_BNO055::VECTOR_EULER);

  serviceLidarRx();
  serviceUsbRx(millis());
  serviceUsbTx();

  bno.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE);

  serviceLidarRx();
  serviceUsbRx(millis());
  serviceUsbTx();

  if (static_cast<uint32_t>(now_ms - last_imu_calibration_ms) >=
      Config::IMU_CALIBRATION_PERIOD_MS) {
    last_imu_calibration_ms = now_ms;

    bno.getCalibration(&imu_cal_system, &imu_cal_gyro, &imu_cal_accel,
                       &imu_cal_mag);

    serviceLidarRx();
    serviceUsbRx(millis());
  }

  const float raw_heading_deg = orientation_event.orientation.x;

  if (!imu_have_heading) {
    imu_last_raw_heading_deg = raw_heading_deg;
    imu_continuous_heading_deg = normalizeHeading360(raw_heading_deg);
    imu_have_heading = true;
    imu_rebase_pending = false;
  } else if (imu_rebase_pending) {
    // Sensor voltou depois de uma falha.
    // A nova referencia bruta e aceita, mas a orientacao entregue ao ROS
    // permanece exatamente onde estava antes da falha.
    imu_last_raw_heading_deg = raw_heading_deg;
    imu_rebase_pending = false;
  } else {
    const float delta_deg =
        shortestHeadingDelta(raw_heading_deg, imu_last_raw_heading_deg);

    uint32_t dt_ms =
        static_cast<uint32_t>(now_ms - last_good_imu_ms);

    if (last_good_imu_ms == 0U || dt_ms == 0U) {
      dt_ms = Config::IMU_PERIOD_MS;
    }

    float allowed_jump_deg =
        IMU_MAX_PLAUSIBLE_RATE_DEG_S *
        (static_cast<float>(dt_ms) / 1000.0F);

    if (allowed_jump_deg < IMU_MIN_ALLOWED_JUMP_DEG) {
      allowed_jump_deg = IMU_MIN_ALLOWED_JUMP_DEG;
    }

    if (fabsf(delta_deg) > allowed_jump_deg) {
      // Exemplo do erro observado:
      // heading real ~ -9.7 graus -> leitura 0 instantaneamente.
      // Esse valor nao chega ao ROS.
      markImuFault();
      return;
    }

    imu_continuous_heading_deg =
        normalizeHeading360(imu_continuous_heading_deg + delta_deg);

    imu_last_raw_heading_deg = raw_heading_deg;
  }

  last_good_imu_ms = now_ms;

  const int32_t heading_mdeg = static_cast<int32_t>(
      lroundf(imu_continuous_heading_deg * 1000.0F));

  const int16_t gyro_z_mrad_s = saturateI16(
      lroundf(gyro_event.gyro.z * 1000.0F));

  const bool timed_imu = temporalEnabled();
  const uint16_t payload_size = timed_imu ? 28U : 16U;
  uint16_t crc = 0;

  if (!usbFrameBegin(timed_imu ? UsbProtocol::TYPE_IMU_TIMED : UsbProtocol::TYPE_IMU, payload_size, crc)) {
    return;
  }

  usbFramePushU16(imu_sequence++, crc);
  usbFramePushU32(now_ms, crc);
  usbFramePushI32(heading_mdeg, crc);
  usbFramePushI16(gyro_z_mrad_s, crc);
  usbFramePush(imu_cal_system, crc);
  usbFramePush(imu_cal_gyro, crc);
  usbFramePush(imu_cal_accel, crc);
  usbFramePush(imu_cal_mag, crc);
  // Begin antecede leituras I2C; fim inclui EULER, gyro e calibracao.
  if (timed_imu) {
    usbFramePushU32(imu_capture_begin_us, crc);
    usbFramePushU32(micros(), crc);
    usbFramePushU32(micros(), crc);
  }
  usbFrameEnd(crc);
}

// ---------- Motor do LiDAR e telemetria ----------

uint32_t lidar_motor_started_ms = 0;
bool lidar_motor_in_startup = true;

void serviceLidarMotor(uint32_t now_ms) {
  if (lidar_motor_in_startup &&
      static_cast<uint32_t>(now_ms - lidar_motor_started_ms) >=
          Config::LIDAR_START_TIME_MS) {
    analogWrite(Config::LIDAR_MOTOR_PWM_PIN, Config::LIDAR_WORK_PWM);
    lidar_motor_in_startup = false;
  }
}

uint16_t saturateU16(uint32_t value) {
  return value > 65535UL ? 65535U : static_cast<uint16_t>(value);
}

void queueStatus(uint32_t now_ms) {
  constexpr uint16_t payload_size = 88U;
  uint16_t crc = 0;
  if (!usbFrameBegin(UsbProtocol::TYPE_STATUS, payload_size, crc)) {
    return;
  }

  usbFramePushU16(last_command_sequence, crc);
  usbFramePushU32(now_ms, crc);
  const uint16_t command_age = command_received
                                   ? saturateU16(now_ms - last_valid_command_ms)
                                   : 65535U;
  usbFramePushU16(command_age, crc);
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    usbFramePushI16(target_wheel_pwm[wheel], crc);
  }
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
    usbFramePushI16(static_cast<int16_t>(lroundf(current_wheel_pwm[wheel])), crc);
  }
  usbFramePush(lidar_scan_freq_x20, crc);
  usbFramePushU16(last_scan_valid_bins, crc);
  usbFramePushU32(lidar_good_packets, crc);
  usbFramePushU32(lidar_bad_checksums, crc);
  usbFramePushU32(lidar_bad_frames, crc);
  usbFramePushU32(usb_rx_bad_frames, crc);
  usbFramePushU32(usb_tx_dropped_frames, crc);
  usbFramePush(watchdog_active ? 1U : 0U, crc);
  usbFramePush(lidar_motor_in_startup ? Config::LIDAR_START_PWM
                                     : Config::LIDAR_WORK_PWM,
               crc);
  usbFramePush(imu_present ? 1U : 0U, crc);
  usbFramePush(imu_cal_system, crc);
  usbFramePush(imu_cal_gyro, crc);
  usbFramePush(imu_cal_accel, crc);
  usbFramePush(imu_cal_mag, crc);

  usbFramePushU32(lidar_bad_timeout, crc);
  usbFramePushU32(lidar_bad_size, crc);
  usbFramePushU32(lidar_bad_version, crc);
  usbFramePushU32(lidar_bad_packet_type, crc);
  usbFramePushU32(lidar_bad_measurement, crc);
  usbFramePushU32(lidar_bad_unknown_type, crc);
  usbFramePushU32(lidar_bad_overflow, crc);
  usbFramePushU16(lidar_max_serial2_available, crc);
  usbFramePushU32(max_loop_gap_us, crc);

  usbFrameEnd(crc);
}

void setupMotorPin(const MotorPins &pins) {
  pinMode(pins.forward_pwm, OUTPUT);
  pinMode(pins.reverse_pwm, OUTPUT);
  analogWrite(pins.forward_pwm, 0);
  analogWrite(pins.reverse_pwm, 0);
}

void setup() {
  setupMotorPin(M1);
  setupMotorPin(M2);
  setupMotorPin(M3);
  setupMotorPin(M4);
  stopDriveImmediately();

  pinMode(Config::LIDAR_MOTOR_PWM_PIN, OUTPUT);
  analogWrite(Config::LIDAR_MOTOR_PWM_PIN, Config::LIDAR_START_PWM);
  lidar_motor_started_ms = millis();

  clearScan();
  Serial2.begin(Config::LIDAR_BAUD);
  Serial.begin(Config::USB_BAUD);

  Wire.begin();
  Wire.setWireTimeout(5000, true);
  
  imu_present = bno.begin();
  if (imu_present) {
    bno.setExtCrystalUse(true);
  }

  const uint32_t now_ms = millis();
  last_motor_update_ms = now_ms;
  last_imu_ms = now_ms;
  last_imu_calibration_ms = now_ms;
}

void loop() {
  const uint32_t loop_entry_us = micros();

  if (last_loop_entry_us != 0U) {
    const uint32_t loop_gap_us =
        static_cast<uint32_t>(loop_entry_us - last_loop_entry_us);

    if (loop_gap_us > max_loop_gap_us) {
      max_loop_gap_us = loop_gap_us;
    }
  }

  last_loop_entry_us = loop_entry_us;

  const uint32_t now_ms = millis();

  serviceLidarRx();
  serviceUsbRx(now_ms);

  // Sem referencia angular confiavel, o robo nao deve continuar navegando.
  if (imu_present) {
    serviceMotors(now_ms);
  } else {
    stopDriveImmediately();
  }

  serviceLidarMotor(now_ms);

  // Libera a fila USB antes das leituras I2C do BNO055.
  serviceUsbTx();
  serviceImu(now_ms);
  queueTemporalSnapshot();

  // Segunda drenagem imediatamente apos a IMU: impede que bytes recebidos
  // durante as transacoes I2C permaneçam acumulados ate a proxima volta.
  serviceLidarRx();
  serviceUsbRx(millis());

  // Detecta e recupera perda de sincronismo, scan travado
  // ou UART silenciosa antes do source_timeout do Nav2.
  serviceLidarHealth(millis());

  static uint32_t last_status_ms = 0;
  if (static_cast<uint32_t>(now_ms - last_status_ms) >= Config::STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    queueStatus(now_ms);
  }

  serviceUsbTx();
}