#include <Arduino.h>
#include <math.h>

// Firmware da base 4WD + LiDAR Delta-2D para Arduino Mega 2560.
// Projeto sem encoders. Todo o processamento e toda a comunicacao sao nao bloqueantes.

namespace Config {

constexpr uint32_t USB_BAUD = 500000UL;
constexpr uint32_t LIDAR_BAUD = 115200UL;

constexpr uint8_t LIDAR_MOTOR_PWM_PIN = 44;
constexpr uint8_t LIDAR_START_PWM = 255;
constexpr uint8_t LIDAR_WORK_PWM = 200;
constexpr uint32_t LIDAR_START_TIME_MS = 2000UL;

constexpr int16_t MOTOR_MIN_PWM = 150;
constexpr int16_t MOTOR_MAX_PWM = 215;
constexpr float MOTOR_ACCEL_PWM_PER_S = 200.0F;  // 0 -> 150 em 0,75 s
constexpr float MOTOR_DECEL_PWM_PER_S = 600.0F;
constexpr uint32_t MOTOR_UPDATE_MS = 5UL;
constexpr uint32_t COMMAND_WATCHDOG_MS = 300UL;

constexpr uint16_t LIDAR_MIN_MM = 80;
constexpr uint16_t LIDAR_MAX_MM = 6000;
constexpr uint16_t SCAN_BIN_COUNT = 180;
constexpr float SCAN_BIN_DEG = 2.0F;
constexpr uint16_t MIN_VALID_SCAN_BINS = 25;
constexpr float SENSOR_FRONT_DEG = 270.0F;

constexpr uint32_t STATUS_PERIOD_MS = 500UL;

}  // namespace Config

struct MotorPins {
  uint8_t forward_pwm;
  uint8_t reverse_pwm;
  bool inverted;
};

// Mapeamento confirmado pelo teste individual em bancada.
// O campo inverted faz PWM positivo sempre representar avanco fisico.
constexpr MotorPins M1{8, 7, true};    // traseira direita
constexpr MotorPins M2{9, 10, true};   // dianteira esquerda
constexpr MotorPins M3{5, 6, true};    // traseira esquerda
constexpr MotorPins M4{11, 12, false}; // dianteira direita

namespace UsbProtocol {

constexpr uint8_t SOF_1 = 0xA5;
constexpr uint8_t SOF_2 = 0x5A;
constexpr uint8_t VERSION = 0x01;

constexpr uint8_t TYPE_MOTOR_COMMAND = 0x01;
constexpr uint8_t TYPE_SCAN = 0x10;
constexpr uint8_t TYPE_STATUS = 0x11;

constexpr uint8_t FLAG_ENABLE = 0x01;
constexpr uint8_t FLAG_ESTOP = 0x02;

constexpr uint16_t MOTOR_COMMAND_PAYLOAD_SIZE = 7;
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
constexpr uint16_t MAX_SAMPLE_COUNT = 24;
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

int16_t target_left_pwm = 0;
int16_t target_right_pwm = 0;
float current_left_pwm = 0.0F;
float current_right_pwm = 0.0F;
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
  if (magnitude < Config::MOTOR_MIN_PWM) {
    magnitude = Config::MOTOR_MIN_PWM;
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

void writeDrive(int16_t left_pwm, int16_t right_pwm) {
  writeSingleMotor(M1, right_pwm);
  writeSingleMotor(M4, right_pwm);
  writeSingleMotor(M2, left_pwm);
  writeSingleMotor(M3, left_pwm);
}

void stopDriveImmediately() {
  target_left_pwm = 0;
  target_right_pwm = 0;
  current_left_pwm = 0.0F;
  current_right_pwm = 0.0F;
  writeDrive(0, 0);
}

float stepMotorRamp(float current, int16_t requested, float elapsed_s) {
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
  current_left_pwm = stepMotorRamp(current_left_pwm, target_left_pwm, elapsed_s);
  current_right_pwm = stepMotorRamp(current_right_pwm, target_right_pwm, elapsed_s);

  writeDrive(static_cast<int16_t>(lroundf(current_left_pwm)),
             static_cast<int16_t>(lroundf(current_right_pwm)));
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

void applyMotorCommand(const uint8_t *payload, uint32_t now_ms) {
  const uint16_t sequence = readU16LE(payload);
  const int16_t left = readI16LE(payload + 2);
  const int16_t right = readI16LE(payload + 4);
  const uint8_t flags = payload[6];

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

  target_left_pwm = sanitizeRequestedPwm(left);
  target_right_pwm = sanitizeRequestedPwm(right);
  watchdog_active = false;
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

  if (usb_rx_buffer[1] == UsbProtocol::TYPE_MOTOR_COMMAND &&
      payload_size == UsbProtocol::MOTOR_COMMAND_PAYLOAD_SIZE) {
    applyMotorCommand(usb_rx_buffer + 4, now_ms);
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

void clearScan() {
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

void processDeltaMeasurement(uint16_t frame_size, uint32_t now_us) {
  const uint16_t packet_length = readU16BE(lidar_frame + 1);
  const uint16_t data_length = readU16BE(lidar_frame + 6);

  if (packet_length + 2U != frame_size || packet_length != data_length + 8U ||
      data_length < 8U || ((data_length - 5U) % 3U) != 0U) {
    ++lidar_bad_frames;
    return;
  }

  const uint16_t sample_count = (data_length - 5U) / 3U;
  if (sample_count == 0U || sample_count > DeltaProtocol::MAX_SAMPLE_COUNT ||
      13U + sample_count * 3U + 2U != frame_size) {
    ++lidar_bad_frames;
    return;
  }

  lidar_scan_freq_x20 = lidar_frame[8];
  const float start_angle_deg = static_cast<float>(readU16BE(lidar_frame + 11)) * 0.01F;

  const bool completed_rotation =
      have_previous_packet_angle && previous_packet_start_deg >= 300.0F &&
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

  const float angular_step_deg =
      360.0F / (static_cast<float>(DeltaProtocol::PACKETS_PER_SCAN) * sample_count);

  for (uint16_t i = 0; i < sample_count; ++i) {
    const uint16_t sample_offset = 13U + i * 3U;
    const uint16_t distance_mm_x4 = readU16BE(lidar_frame + sample_offset + 1U);
    const uint16_t distance_mm = static_cast<uint16_t>(distance_mm_x4 / 4U);

    if (distance_mm < Config::LIDAR_MIN_MM || distance_mm > Config::LIDAR_MAX_MM) {
      continue;
    }

    float sensor_angle_deg = start_angle_deg + static_cast<float>(i) * angular_step_deg;
    while (sensor_angle_deg >= 360.0F) {
      sensor_angle_deg -= 360.0F;
    }

    // Delta-2D invertido: 270 graus do sensor corresponde a frente do robo.
    const float robot_angle_deg =
        normalizeAngle180(Config::SENSOR_FRONT_DEG - sensor_angle_deg);
    int16_t bin = static_cast<int16_t>(floorf((robot_angle_deg + 180.0F) /
                                              Config::SCAN_BIN_DEG));
    if (bin < 0) {
      bin = 0;
    } else if (bin >= static_cast<int16_t>(Config::SCAN_BIN_COUNT)) {
      bin = Config::SCAN_BIN_COUNT - 1;
    }

    uint16_t &stored_mm = scan_ranges_mm[bin];
    if (stored_mm == 0U) {
      stored_mm = distance_mm;
      ++scan_valid_bins;
    } else if (distance_mm < stored_mm) {
      stored_mm = distance_mm;
    }
  }
}

void processCompleteDeltaFrame(uint16_t frame_size, uint32_t now_us) {
  if (!validateDeltaChecksum(frame_size)) {
    ++lidar_bad_checksums;
    return;
  }

  if (lidar_frame[3] != DeltaProtocol::VERSION ||
      lidar_frame[4] != DeltaProtocol::PACKET_TYPE) {
    ++lidar_bad_frames;
    return;
  }

  if (lidar_frame[5] == DeltaProtocol::DATA_RPM_AND_MEAS) {
    processDeltaMeasurement(frame_size, now_us);
  } else if (lidar_frame[5] == DeltaProtocol::DATA_RPM_ONLY) {
    if (frame_size >= DeltaProtocol::MIN_FRAME_SIZE) {
      lidar_scan_freq_x20 = lidar_frame[8];
    }
  } else {
    ++lidar_bad_frames;
    return;
  }

  ++lidar_good_packets;
}

void processLidarByte(uint8_t value, uint32_t now_ms, uint32_t now_us) {
  if (lidar_frame_index != 0U &&
      static_cast<uint32_t>(now_ms - lidar_last_byte_ms) >
          DeltaProtocol::INTERBYTE_TIMEOUT_MS) {
    ++lidar_bad_frames;
    resetLidarParser();
  }
  lidar_last_byte_ms = now_ms;

  if (lidar_frame_index == 0U && value != DeltaProtocol::START_BYTE) {
    return;
  }

  if (lidar_frame_index >= DeltaProtocol::MAX_FRAME_SIZE) {
    ++lidar_bad_frames;
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
      resetLidarParser();
      return;
    }
  }

  if (lidar_frame_index == 4U && lidar_frame[3] != DeltaProtocol::VERSION) {
    ++lidar_bad_frames;
    resetLidarParser();
    return;
  }

  if (lidar_frame_index == 5U && lidar_frame[4] != DeltaProtocol::PACKET_TYPE) {
    ++lidar_bad_frames;
    resetLidarParser();
    return;
  }

  if (lidar_expected_size != 0U && lidar_frame_index == lidar_expected_size) {
    processCompleteDeltaFrame(lidar_expected_size, now_us);
    resetLidarParser();
  }
}

void serviceLidarRx() {
  uint16_t processed = 0;
  while (Serial2.available() > 0 && processed < 96U) {
    const uint32_t now_ms = millis();
    const uint32_t now_us = micros();
    processLidarByte(static_cast<uint8_t>(Serial2.read()), now_ms, now_us);
    ++processed;
  }
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
  constexpr uint16_t payload_size = 41U;
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
  usbFramePushI16(target_left_pwm, crc);
  usbFramePushI16(target_right_pwm, crc);
  usbFramePushI16(static_cast<int16_t>(lroundf(current_left_pwm)), crc);
  usbFramePushI16(static_cast<int16_t>(lroundf(current_right_pwm)), crc);
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

  const uint32_t now_ms = millis();
  last_motor_update_ms = now_ms;
}

void loop() {
  const uint32_t now_ms = millis();

  serviceLidarRx();
  serviceUsbRx(now_ms);
  serviceMotors(now_ms);
  serviceLidarMotor(now_ms);

  static uint32_t last_status_ms = 0;
  if (static_cast<uint32_t>(now_ms - last_status_ms) >= Config::STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    queueStatus(now_ms);
  }

  serviceUsbTx();
}
