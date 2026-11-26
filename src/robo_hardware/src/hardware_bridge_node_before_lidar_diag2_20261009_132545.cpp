#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

using namespace std::chrono_literals;

constexpr uint8_t kSof1 = 0xA5;
constexpr uint8_t kSof2 = 0x5A;
constexpr uint8_t kProtocolVersion = 0x02;
constexpr uint8_t kTypeMecanumCommand = 0x01;
constexpr uint8_t kTypeScan = 0x10;
constexpr uint8_t kTypeStatus = 0x11;
constexpr uint8_t kTypeImu = 0x12;
constexpr uint8_t kFlagEnable = 0x01;
constexpr uint8_t kFlagEstop = 0x02;
constexpr std::size_t kScanBinCount = 180;
constexpr std::size_t kScanPayloadSize = 373;
constexpr std::size_t kStatusPayloadSize = 54;
constexpr std::size_t kImuPayloadSize = 16;
constexpr std::size_t kMaxPayloadSize = 512;
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int16_t read_i16_le(const uint8_t * data)
{
  return static_cast<int16_t>(read_u16_le(data));
}

uint32_t read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t read_i32_le(const uint8_t * data)
{
  return static_cast<int32_t>(read_u32_le(data));
}

void append_u16_le(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
}

void append_i16_le(std::vector<uint8_t> & output, int16_t value)
{
  append_u16_le(output, static_cast<uint16_t>(value));
}

uint16_t crc16_ccitt_update(uint16_t crc, uint8_t value)
{
  crc ^= static_cast<uint16_t>(value) << 8;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x8000U) != 0U ?
      static_cast<uint16_t>((crc << 1) ^ 0x1021U) :
      static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

uint16_t crc16_ccitt(const uint8_t * data, std::size_t size)
{
  uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc = crc16_ccitt_update(crc, data[i]);
  }
  return crc;
}

diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

speed_t baud_to_termios(int baud)
{
  switch (baud) {
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B500000
    case 500000:
      return B500000;
#endif
    default:
      throw std::runtime_error("Unsupported serial baud: " + std::to_string(baud));
  }
}

}  // namespace

class HardwareBridgeNode final : public rclcpp::Node
{
public:
  HardwareBridgeNode()
  : Node("robo_hardware"), steady_clock_(RCL_STEADY_TIME)
  {
    serial_port_ = declare_parameter<std::string>(
      "serial_port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    serial_baud_ = declare_parameter<int>("serial_baud", 500000);
    serial_startup_delay_s_ = declare_parameter<double>("serial_startup_delay_s", 5.0);
    robot_length_m_ = declare_parameter<double>("robot_length_m", 0.340);
    robot_width_m_ = declare_parameter<double>("robot_width_m", 0.615);
    maximum_wheel_speed_mps_ = declare_parameter<double>("maximum_wheel_speed_mps", 0.450);
    velocity_deadband_mps_ = declare_parameter<double>("velocity_deadband_mps", 0.005);
    minimum_pwm_ = declare_parameter<int>("minimum_pwm", 150);
    minimum_rotation_pwm_ = declare_parameter<int>("minimum_rotation_pwm", 100);
    maximum_pwm_ = declare_parameter<int>("maximum_pwm", 215);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 20.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.25);
    scan_frame_ = declare_parameter<std::string>("scan_frame", "laser_raw");
    scan_range_min_m_ = declare_parameter<double>("scan_range_min_m", 0.08);
    scan_range_max_m_ = declare_parameter<double>("scan_range_max_m", 6.0);
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    imu_yaw_sign_ = declare_parameter<double>("imu_yaw_sign", -1.0);
    imu_yaw_offset_deg_ = declare_parameter<double>("imu_yaw_offset_deg", 0.0);
    zero_imu_on_start_ = declare_parameter<bool>("zero_imu_on_start", true);
    heading_hold_enabled_ = declare_parameter<bool>("heading_hold_enabled", true);
    heading_hold_kp_ = declare_parameter<double>("heading_hold_kp", 0.8);
    heading_hold_kd_ = declare_parameter<double>("heading_hold_kd", 0.05);
    heading_hold_max_correction_rps_ = declare_parameter<double>(
      "heading_hold_max_correction_rps", 0.15);
    heading_hold_lateral_kp_ = declare_parameter<double>("heading_hold_lateral_kp", 1.2);
    heading_hold_lateral_kd_ = declare_parameter<double>("heading_hold_lateral_kd", 0.08);
    heading_hold_lateral_max_correction_rps_ = declare_parameter<double>(
      "heading_hold_lateral_max_correction_rps", 0.15);
    heading_hold_lateral_feedforward_rps_ = declare_parameter<double>(
      "heading_hold_lateral_feedforward_rps", 0.025);
    heading_hold_error_deadband_deg_ = declare_parameter<double>(
      "heading_hold_error_deadband_deg", 0.5);
    heading_hold_linear_authority_ratio_ = declare_parameter<double>(
      "heading_hold_linear_authority_ratio", 0.75);
    heading_hold_linear_deadband_mps_ = declare_parameter<double>(
      "heading_hold_linear_deadband_mps", 0.01);
    heading_hold_angular_deadband_rps_ = declare_parameter<double>(
      "heading_hold_angular_deadband_rps", 0.02);
    heading_hold_imu_timeout_s_ = declare_parameter<double>(
      "heading_hold_imu_timeout_s", 0.20);

    validate_parameters();

    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return update_heading_hold_parameters(parameters);
      });

    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_raw", rclcpp::SensorDataQoS().keep_last(5));
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS().keep_last(20));
    yaw_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "imu/yaw_deg", rclcpp::QoS(10).reliable());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10).reliable());

    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        receive_command(*message);
      });

    emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        emergency_stop_ = message->data;
        if (emergency_stop_) {
          have_velocity_command_ = false;
          reset_heading_hold();
          queue_motor_command({0, 0, 0, 0}, kFlagEstop);
          RCLCPP_WARN(get_logger(), "Emergency stop active");
        } else {
          RCLCPP_INFO(get_logger(), "Emergency stop released");
        }
      });

    io_timer_ = create_wall_timer(2ms, [this]() { service_serial(); });
    const auto command_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_rate_hz_));
    command_timer_ = create_wall_timer(command_period, [this]() { send_periodic_command(); });
    diagnostic_timer_ = create_wall_timer(1s, [this]() { monitor_connection(); });

    next_reconnect_time_ = steady_clock_.now();
    RCLCPP_INFO(
      get_logger(),
      "Mecanum+BNO055 bridge ready: port=%s baud=%d heading_hold=%s",
      serial_port_.c_str(), serial_baud_, heading_hold_enabled_ ? "on" : "off");
  }

  ~HardwareBridgeNode() override
  {
    send_emergency_stop_best_effort();
    close_serial();
  }

private:
  struct MegaStatus
  {
    uint16_t command_sequence{0};
    uint32_t uptime_ms{0};
    uint16_t command_age_ms{65535};
    std::array<int16_t, 4> target_pwm{{0, 0, 0, 0}};
    std::array<int16_t, 4> current_pwm{{0, 0, 0, 0}};
    uint8_t scan_frequency_x20{0};
    uint16_t valid_scan_bins{0};
    uint32_t good_lidar_packets{0};
    uint32_t bad_lidar_checksums{0};
    uint32_t bad_lidar_frames{0};
    uint32_t bad_usb_rx_frames{0};
    uint32_t dropped_usb_tx_frames{0};
    bool watchdog_active{true};
    uint8_t lidar_motor_pwm{0};
    bool imu_present{false};
    std::array<uint8_t, 4> imu_calibration{{0, 0, 0, 0}};
  };

  void validate_parameters()
  {
    if (serial_port_.empty()) {
      throw std::invalid_argument("serial_port must not be empty");
    }
    if (serial_startup_delay_s_ < 0.0 || serial_startup_delay_s_ > 30.0) {
      throw std::invalid_argument("serial_startup_delay_s must be between 0 and 30 seconds");
    }
    if (robot_length_m_ <= 0.0 || robot_width_m_ <= 0.0 ||
      maximum_wheel_speed_mps_ <= 0.0)
    {
      throw std::invalid_argument("wheel geometry and maximum speed must be positive");
    }
    if (minimum_pwm_ <= 0 || minimum_rotation_pwm_ <= 0 ||
      maximum_pwm_ > 255 || minimum_pwm_ >= maximum_pwm_ ||
      minimum_rotation_pwm_ >= maximum_pwm_)
    {
      throw std::invalid_argument("invalid PWM limits");
    }
    if (command_rate_hz_ <= 0.0 || command_timeout_s_ <= 0.0) {
      throw std::invalid_argument("command timing parameters must be positive");
    }
    if (scan_range_min_m_ <= 0.0 || scan_range_max_m_ <= scan_range_min_m_) {
      throw std::invalid_argument("invalid scan range");
    }
    if (imu_yaw_sign_ != -1.0 && imu_yaw_sign_ != 1.0) {
      throw std::invalid_argument("imu_yaw_sign must be -1.0 or 1.0");
    }
    if (!heading_hold_parameters_valid(
        heading_hold_kp_, heading_hold_kd_, heading_hold_max_correction_rps_,
        heading_hold_lateral_kp_, heading_hold_lateral_kd_,
        heading_hold_lateral_max_correction_rps_,
        heading_hold_lateral_feedforward_rps_, heading_hold_error_deadband_deg_,
        heading_hold_linear_authority_ratio_, heading_hold_linear_deadband_mps_,
        heading_hold_angular_deadband_rps_, heading_hold_imu_timeout_s_))
    {
      throw std::invalid_argument("invalid heading hold parameters");
    }
  }

  static bool heading_hold_parameters_valid(
    double kp, double kd, double max_correction, double lateral_kp,
    double lateral_kd, double lateral_max_correction, double lateral_feedforward,
    double error_deadband_deg, double linear_authority_ratio,
    double linear_deadband, double angular_deadband, double imu_timeout)
  {
    return kp >= 0.0 && kd >= 0.0 && max_correction > 0.0 &&
           lateral_kp >= 0.0 && lateral_kd >= 0.0 &&
           lateral_max_correction > 0.0 && lateral_feedforward >= 0.0 &&
           error_deadband_deg >= 0.0 && error_deadband_deg <= 10.0 &&
           linear_authority_ratio > 0.0 && linear_authority_ratio <= 1.0 &&
           linear_deadband >= 0.0 && angular_deadband >= 0.0 && imu_timeout > 0.0;
  }

  rcl_interfaces::msg::SetParametersResult update_heading_hold_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    bool enabled = heading_hold_enabled_;
    double kp = heading_hold_kp_;
    double kd = heading_hold_kd_;
    double max_correction = heading_hold_max_correction_rps_;
    double lateral_kp = heading_hold_lateral_kp_;
    double lateral_kd = heading_hold_lateral_kd_;
    double lateral_max_correction = heading_hold_lateral_max_correction_rps_;
    double lateral_feedforward = heading_hold_lateral_feedforward_rps_;
    double error_deadband_deg = heading_hold_error_deadband_deg_;
    double linear_authority_ratio = heading_hold_linear_authority_ratio_;
    double linear_deadband = heading_hold_linear_deadband_mps_;
    double angular_deadband = heading_hold_angular_deadband_rps_;
    double imu_timeout = heading_hold_imu_timeout_s_;

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    try {
      for (const auto & parameter : parameters) {
        const std::string & name = parameter.get_name();
        if (name == "heading_hold_enabled") {
          enabled = parameter.as_bool();
        } else if (name == "heading_hold_kp") {
          kp = parameter.as_double();
        } else if (name == "heading_hold_kd") {
          kd = parameter.as_double();
        } else if (name == "heading_hold_max_correction_rps") {
          max_correction = parameter.as_double();
        } else if (name == "heading_hold_lateral_kp") {
          lateral_kp = parameter.as_double();
        } else if (name == "heading_hold_lateral_kd") {
          lateral_kd = parameter.as_double();
        } else if (name == "heading_hold_lateral_max_correction_rps") {
          lateral_max_correction = parameter.as_double();
        } else if (name == "heading_hold_lateral_feedforward_rps") {
          lateral_feedforward = parameter.as_double();
        } else if (name == "heading_hold_error_deadband_deg") {
          error_deadband_deg = parameter.as_double();
        } else if (name == "heading_hold_linear_authority_ratio") {
          linear_authority_ratio = parameter.as_double();
        } else if (name == "heading_hold_linear_deadband_mps") {
          linear_deadband = parameter.as_double();
        } else if (name == "heading_hold_angular_deadband_rps") {
          angular_deadband = parameter.as_double();
        } else if (name == "heading_hold_imu_timeout_s") {
          imu_timeout = parameter.as_double();
        }
      }
    } catch (const std::exception & exception) {
      result.reason = exception.what();
      return result;
    }

    if (!heading_hold_parameters_valid(
        kp, kd, max_correction, lateral_kp, lateral_kd,
        lateral_max_correction, lateral_feedforward, error_deadband_deg,
        linear_authority_ratio, linear_deadband, angular_deadband, imu_timeout))
    {
      result.reason = "invalid heading hold value";
      return result;
    }

    heading_hold_enabled_ = enabled;
    heading_hold_kp_ = kp;
    heading_hold_kd_ = kd;
    heading_hold_max_correction_rps_ = max_correction;
    heading_hold_lateral_kp_ = lateral_kp;
    heading_hold_lateral_kd_ = lateral_kd;
    heading_hold_lateral_max_correction_rps_ = lateral_max_correction;
    heading_hold_lateral_feedforward_rps_ = lateral_feedforward;
    heading_hold_error_deadband_deg_ = error_deadband_deg;
    heading_hold_linear_authority_ratio_ = linear_authority_ratio;
    heading_hold_linear_deadband_mps_ = linear_deadband;
    heading_hold_angular_deadband_rps_ = angular_deadband;
    heading_hold_imu_timeout_s_ = imu_timeout;
    reset_heading_hold();

    result.successful = true;
    result.reason = "heading hold parameters updated";
    return result;
  }

  void receive_command(const geometry_msgs::msg::TwistStamped & message)
  {
    const double linear_x = message.twist.linear.x;
    const double linear_y = message.twist.linear.y;
    const double angular_z = message.twist.angular.z;
    if (!std::isfinite(linear_x) || !std::isfinite(linear_y) ||
      !std::isfinite(angular_z))
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite velocity command");
      return;
    }

    commanded_vx_mps_ = linear_x;
    commanded_vy_mps_ = linear_y;
    commanded_wz_rps_ = angular_z;
    last_velocity_command_time_ = steady_clock_.now();
    have_velocity_command_ = true;
  }

  bool open_serial()
  {
    const int file_descriptor = open(
      serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (file_descriptor < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unable to open %s: %s", serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    termios settings{};
    if (tcgetattr(file_descriptor, &settings) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      close(file_descriptor);
      return false;
    }

    try {
      const speed_t baud = baud_to_termios(serial_baud_);
      cfmakeraw(&settings);
      cfsetispeed(&settings, baud);
      cfsetospeed(&settings, baud);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "%s", exception.what());
      close(file_descriptor);
      return false;
    }

    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
    settings.c_cflag |= CS8;
#ifdef CRTSCTS
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    // Configuracao comprovada no teste bidirecional do CH340. poll() impede
    // a leitura ociosa; VMIN=0 preserva a semantica nao bloqueante validada.
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (tcsetattr(file_descriptor, TCSANOW, &settings) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      close(file_descriptor);
      return false;
    }

    tcflush(file_descriptor, TCIOFLUSH);
    serial_fd_ = file_descriptor;
    serial_open_time_ = steady_clock_.now();
    serial_command_ready_time_ = steady_clock_.now() +
      rclcpp::Duration::from_seconds(serial_startup_delay_s_);
    reset_rx_parser();
    pending_tx_frame_.clear();
    pending_tx_offset_ = 0;
    RCLCPP_INFO(
      get_logger(), "Serial connected: %s; command TX starts after %.1f s",
      serial_port_.c_str(), serial_startup_delay_s_);
    return true;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    reset_rx_parser();
    pending_tx_frame_.clear();
    pending_tx_offset_ = 0;
    have_latest_imu_ = false;
    reset_heading_hold();
  }

  void handle_serial_error(const char * operation)
  {
    RCLCPP_ERROR(
      get_logger(), "Serial %s failed on %s: %s",
      operation, serial_port_.c_str(), std::strerror(errno));
    close_serial();
    next_reconnect_time_ = steady_clock_.now() + rclcpp::Duration::from_seconds(1.0);
  }

  void service_serial()
  {
    if (serial_fd_ < 0) {
      if (steady_clock_.now() >= next_reconnect_time_) {
        if (!open_serial()) {
          next_reconnect_time_ = steady_clock_.now() + rclcpp::Duration::from_seconds(1.0);
        }
      }
      return;
    }

    pollfd descriptor{};
    descriptor.fd = serial_fd_;
    descriptor.events = POLLIN;

    const int poll_result = poll(&descriptor, 1, 0);
    if (poll_result < 0) {
      if (errno != EINTR) {
        handle_serial_error("poll");
      }
      return;
    }

    if (poll_result > 0 && (descriptor.revents & POLLNVAL) != 0) {
      errno = EBADF;
      handle_serial_error("poll");
      return;
    }

    std::array<uint8_t, 2048> read_buffer{};
    for (int iteration = 0; iteration < 4; ++iteration) {
      if (iteration > 0) {
        descriptor.revents = 0;
        const int more_data = poll(&descriptor, 1, 0);
        if (more_data < 0) {
          if (errno != EINTR) {
            handle_serial_error("poll");
          }
          return;
        }
        if (more_data == 0 || (descriptor.revents & POLLIN) == 0) {
          break;
        }
      } else if (poll_result == 0 || (descriptor.revents & POLLIN) == 0) {
        break;
      }

      const ssize_t count = read(serial_fd_, read_buffer.data(), read_buffer.size());
      if (count > 0) {
        if (host_rx_bytes_ == 0U) {
          RCLCPP_INFO(
            get_logger(), "First serial RX after %.3f s (%zd bytes)",
            (steady_clock_.now() - serial_open_time_).seconds(), count);
        }
        host_rx_bytes_ += static_cast<uint64_t>(count);
        ++host_rx_reads_;
        for (ssize_t i = 0; i < count; ++i) {
          process_rx_byte(read_buffer[static_cast<std::size_t>(i)]);
        }
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      handle_serial_error("read");
      return;
    }

    flush_serial_tx();
  }

  void reset_rx_parser()
  {
    rx_frame_.clear();
    rx_expected_size_ = 0;
    rx_receiving_ = false;
    rx_saw_first_sof_ = false;
  }

  void process_rx_byte(uint8_t value)
  {
    if (!rx_receiving_) {
      if (!rx_saw_first_sof_) {
        rx_saw_first_sof_ = value == kSof1;
        return;
      }
      if (value == kSof2) {
        rx_receiving_ = true;
        rx_saw_first_sof_ = false;
        rx_frame_.clear();
        rx_expected_size_ = 0;
        return;
      }
      rx_saw_first_sof_ = value == kSof1;
      return;
    }

    rx_frame_.push_back(value);
    if (rx_frame_.size() == 4U) {
      const std::size_t payload_size = read_u16_le(rx_frame_.data() + 2);
      if (payload_size > kMaxPayloadSize) {
        ++host_invalid_frames_;
        reset_rx_parser();
        return;
      }
      rx_expected_size_ = 4U + payload_size + 2U;
    }

    if (rx_expected_size_ != 0U && rx_frame_.size() == rx_expected_size_) {
      process_complete_rx_frame();
      reset_rx_parser();
    } else if (rx_frame_.size() > 4U + kMaxPayloadSize + 2U) {
      ++host_invalid_frames_;
      reset_rx_parser();
    }
  }

  void process_complete_rx_frame()
  {
    const std::size_t payload_size = read_u16_le(rx_frame_.data() + 2);
    const std::size_t crc_offset = 4U + payload_size;
    const uint16_t received_crc = read_u16_le(rx_frame_.data() + crc_offset);
    const uint16_t calculated_crc = crc16_ccitt(rx_frame_.data(), crc_offset);

    if (received_crc != calculated_crc) {
      ++host_bad_crc_frames_;
      return;
    }
    if (rx_frame_[0] != kProtocolVersion) {
      ++host_invalid_frames_;
      return;
    }

    const uint8_t type = rx_frame_[1];
    const uint8_t * payload = rx_frame_.data() + 4;
    if (type == kTypeScan && payload_size == kScanPayloadSize) {
      publish_scan(payload);
    } else if (type == kTypeStatus && payload_size == kStatusPayloadSize) {
      publish_status(payload);
    } else if (type == kTypeImu && payload_size == kImuPayloadSize) {
      publish_imu(payload);
    } else {
      ++host_invalid_frames_;
    }
  }

  void publish_scan(const uint8_t * payload)
  {
    const uint32_t scan_time_us = read_u32_le(payload + 6);
    const uint8_t frequency_x20 = payload[10];

    double scan_time_s = static_cast<double>(scan_time_us) * 1.0e-6;
    if (!std::isfinite(scan_time_s) || scan_time_s < 0.05 || scan_time_s > 1.0) {
      const double frequency_hz = static_cast<double>(frequency_x20) * 0.05;
      scan_time_s = frequency_hz > 0.1 ? 1.0 / frequency_hz : 0.1666667;
    }

    sensor_msgs::msg::LaserScan message;
    message.header.stamp = now();
    message.header.frame_id = scan_frame_;
    message.angle_min = static_cast<float>(-kPi);
    message.angle_increment = static_cast<float>((2.0 * kPi) / kScanBinCount);
    message.angle_max = message.angle_min +
      static_cast<float>(kScanBinCount - 1U) * message.angle_increment;
    message.scan_time = static_cast<float>(scan_time_s);
    message.time_increment = message.scan_time / static_cast<float>(kScanBinCount);
    message.range_min = static_cast<float>(scan_range_min_m_);
    message.range_max = static_cast<float>(scan_range_max_m_);
    message.ranges.resize(kScanBinCount);

    const uint8_t * ranges = payload + 13;
    for (std::size_t index = 0; index < kScanBinCount; ++index) {
      const uint16_t distance_mm = read_u16_le(ranges + 2U * index);
      const float distance_m = static_cast<float>(distance_mm) * 0.001F;
      if (distance_mm == 0U || distance_m < message.range_min || distance_m > message.range_max) {
        message.ranges[index] = std::numeric_limits<float>::infinity();
      } else {
        message.ranges[index] = distance_m;
      }
    }

    scan_publisher_->publish(message);
    ++published_scans_;
  }

  void publish_status(const uint8_t * payload)
  {
    MegaStatus status;
    status.command_sequence = read_u16_le(payload);
    status.uptime_ms = read_u32_le(payload + 2);
    status.command_age_ms = read_u16_le(payload + 6);
    for (std::size_t index = 0; index < 4U; ++index) {
      status.target_pwm[index] = read_i16_le(payload + 8U + 2U * index);
      status.current_pwm[index] = read_i16_le(payload + 16U + 2U * index);
    }
    status.scan_frequency_x20 = payload[24];
    status.valid_scan_bins = read_u16_le(payload + 25);
    status.good_lidar_packets = read_u32_le(payload + 27);
    status.bad_lidar_checksums = read_u32_le(payload + 31);
    status.bad_lidar_frames = read_u32_le(payload + 35);
    status.bad_usb_rx_frames = read_u32_le(payload + 39);
    status.dropped_usb_tx_frames = read_u32_le(payload + 43);
    status.watchdog_active = payload[47] != 0U;
    status.lidar_motor_pwm = payload[48];
    status.imu_present = payload[49] != 0U;
    for (std::size_t index = 0; index < 4U; ++index) {
      status.imu_calibration[index] = payload[50U + index];
    }

    last_mega_status_ = status;
    have_mega_status_ = true;
    last_status_time_ = steady_clock_.now();
    publish_diagnostics(status);
  }

  void publish_imu(const uint8_t * payload)
  {
    const int32_t heading_mdeg = read_i32_le(payload + 6);
    const int16_t gyro_z_mrad_s = read_i16_le(payload + 10);

    const double raw_yaw_rad = std::atan2(
      std::sin(imu_yaw_sign_ * static_cast<double>(heading_mdeg) * kPi / 180000.0),
      std::cos(imu_yaw_sign_ * static_cast<double>(heading_mdeg) * kPi / 180000.0));

    if (!have_yaw_reference_) {
      yaw_reference_rad_ = zero_imu_on_start_ ? raw_yaw_rad : 0.0;
      have_yaw_reference_ = true;
    }

    const double yaw_rad = std::atan2(
      std::sin(raw_yaw_rad - yaw_reference_rad_ + imu_yaw_offset_deg_ * kPi / 180.0),
      std::cos(raw_yaw_rad - yaw_reference_rad_ + imu_yaw_offset_deg_ * kPi / 180.0));
    const double gyro_z_rad_s =
      imu_yaw_sign_ * static_cast<double>(gyro_z_mrad_s) * 0.001;

    latest_yaw_rad_ = yaw_rad;
    latest_gyro_z_rad_s_ = gyro_z_rad_s;
    latest_imu_time_ = steady_clock_.now();
    have_latest_imu_ = true;

    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = imu_frame_;
    message.orientation.z = std::sin(yaw_rad * 0.5);
    message.orientation.w = std::cos(yaw_rad * 0.5);
    message.orientation_covariance.fill(0.0);
    message.orientation_covariance[0] = 1.0e6;
    message.orientation_covariance[4] = 1.0e6;
    message.orientation_covariance[8] = 0.02;
    message.angular_velocity.z = gyro_z_rad_s;
    message.angular_velocity_covariance.fill(0.0);
    message.angular_velocity_covariance[0] = 1.0e6;
    message.angular_velocity_covariance[4] = 1.0e6;
    message.angular_velocity_covariance[8] = 0.02;
    message.linear_acceleration_covariance[0] = -1.0;
    imu_publisher_->publish(message);

    std_msgs::msg::Float64 yaw_message;
    yaw_message.data = yaw_rad * 180.0 / kPi;
    yaw_publisher_->publish(yaw_message);
    ++published_imu_;
  }

  void publish_diagnostics(const MegaStatus & status)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = "robo_hardware/mega_delta2d_bno055";
    item.hardware_id = "arduino_mega_2560_ch340";

    if (!status.imu_present) {
      item.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      item.message = "BNO055 not detected at 0x29";
    } else if (status.bad_lidar_checksums > 0U || status.bad_lidar_frames > 0U ||
      status.dropped_usb_tx_frames > 0U)
    {
      item.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      item.message = "Running with communication errors";
    } else if (status.imu_calibration[1] < 2U) {
      item.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      item.message = "BNO055 detected; gyroscope calibrating";
    } else if (status.watchdog_active) {
      item.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      item.message = "Connected; motors safely stopped";
    } else {
      item.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      item.message = "Connected and active";
    }

    const double scan_frequency_hz = static_cast<double>(status.scan_frequency_x20) * 0.05;
    const uint32_t scan_rpm = static_cast<uint32_t>(status.scan_frequency_x20) * 3U;

    item.values.push_back(key_value("serial_port", serial_port_));
    item.values.push_back(key_value("protocol_version", std::to_string(kProtocolVersion)));
    item.values.push_back(key_value("mega_uptime_ms", std::to_string(status.uptime_ms)));
    item.values.push_back(key_value("command_sequence", std::to_string(status.command_sequence)));
    item.values.push_back(key_value("command_age_ms", std::to_string(status.command_age_ms)));
    constexpr std::array<const char *, 4> wheel_names{{"FL", "FR", "RL", "RR"}};
    for (std::size_t index = 0; index < wheel_names.size(); ++index) {
      item.values.push_back(key_value(
        std::string("target_pwm_") + wheel_names[index],
        std::to_string(status.target_pwm[index])));
      item.values.push_back(key_value(
        std::string("current_pwm_") + wheel_names[index],
        std::to_string(status.current_pwm[index])));
    }
    item.values.push_back(key_value("lidar_scan_hz", std::to_string(scan_frequency_hz)));
    item.values.push_back(key_value("lidar_rpm", std::to_string(scan_rpm)));
    item.values.push_back(key_value("valid_scan_bins", std::to_string(status.valid_scan_bins)));
    item.values.push_back(key_value("good_lidar_packets", std::to_string(status.good_lidar_packets)));
    item.values.push_back(key_value("bad_lidar_checksums", std::to_string(status.bad_lidar_checksums)));
    item.values.push_back(key_value("bad_lidar_frames", std::to_string(status.bad_lidar_frames)));
    item.values.push_back(key_value("bad_usb_rx_frames", std::to_string(status.bad_usb_rx_frames)));
    item.values.push_back(key_value("dropped_usb_tx_frames", std::to_string(status.dropped_usb_tx_frames)));
    item.values.push_back(key_value("host_bad_crc_frames", std::to_string(host_bad_crc_frames_)));
    item.values.push_back(key_value("host_invalid_frames", std::to_string(host_invalid_frames_)));
    item.values.push_back(key_value("published_scans", std::to_string(published_scans_)));
    item.values.push_back(key_value("published_imu", std::to_string(published_imu_)));
    item.values.push_back(key_value("host_rx_bytes", std::to_string(host_rx_bytes_)));
    item.values.push_back(key_value("host_rx_reads", std::to_string(host_rx_reads_)));
    item.values.push_back(key_value("host_tx_bytes", std::to_string(host_tx_bytes_)));
    item.values.push_back(key_value("watchdog_active", status.watchdog_active ? "true" : "false"));
    item.values.push_back(key_value("lidar_motor_pwm", std::to_string(status.lidar_motor_pwm)));
    item.values.push_back(key_value("imu_present", status.imu_present ? "true" : "false"));
    item.values.push_back(key_value("imu_cal_system", std::to_string(status.imu_calibration[0])));
    item.values.push_back(key_value("imu_cal_gyro", std::to_string(status.imu_calibration[1])));
    item.values.push_back(key_value("imu_cal_accel", std::to_string(status.imu_calibration[2])));
    item.values.push_back(key_value("imu_cal_mag", std::to_string(status.imu_calibration[3])));
    item.values.push_back(key_value(
      "heading_hold_enabled", heading_hold_enabled_ ? "true" : "false"));
    item.values.push_back(key_value(
      "heading_hold_active", heading_hold_active_ ? "true" : "false"));
    item.values.push_back(key_value("heading_hold_mode", heading_hold_mode_));
    item.values.push_back(key_value(
      "heading_hold_target_deg",
      std::to_string(heading_hold_target_rad_ * 180.0 / kPi)));
    item.values.push_back(key_value(
      "heading_hold_error_deg",
      std::to_string(heading_hold_error_rad_ * 180.0 / kPi)));
    item.values.push_back(key_value(
      "heading_hold_correction_rps", std::to_string(heading_hold_correction_rps_)));
    item.values.push_back(key_value(
      "heading_hold_feedback_rps", std::to_string(heading_hold_feedback_rps_)));
    item.values.push_back(key_value(
      "heading_hold_feedforward_rps", std::to_string(heading_hold_feedforward_rps_)));
    item.values.push_back(key_value(
      "heading_hold_active_kp", std::to_string(heading_hold_active_kp_)));
    item.values.push_back(key_value(
      "heading_hold_active_kd", std::to_string(heading_hold_active_kd_)));
    item.values.push_back(key_value(
      "heading_hold_effective_limit_rps",
      std::to_string(heading_hold_effective_limit_rps_)));
    item.values.push_back(key_value(
      "heading_hold_activations", std::to_string(heading_hold_activations_)));
    item.values.push_back(key_value(
      "latest_yaw_deg", std::to_string(latest_yaw_rad_ * 180.0 / kPi)));
    item.values.push_back(key_value(
      "imu_age_s",
      have_latest_imu_ ?
      std::to_string((steady_clock_.now() - latest_imu_time_).seconds()) : "unavailable"));

    array.status.push_back(std::move(item));
    diagnostics_publisher_->publish(array);
  }

  int16_t wheel_speed_to_pwm(double speed_mps, int effective_minimum_pwm) const
  {
    const double magnitude = std::abs(speed_mps);
    if (magnitude < velocity_deadband_mps_) {
      return 0;
    }

    const double ratio = std::clamp(magnitude / maximum_wheel_speed_mps_, 0.0, 1.0);
    const double pwm = static_cast<double>(effective_minimum_pwm) +
      ratio * static_cast<double>(maximum_pwm_ - effective_minimum_pwm);
    const int16_t signed_pwm = static_cast<int16_t>(std::lround(pwm));
    return speed_mps < 0.0 ? static_cast<int16_t>(-signed_pwm) : signed_pwm;
  }

  void reset_heading_hold()
  {
    heading_hold_active_ = false;
    heading_hold_mode_ = "inactive";
    heading_hold_error_rad_ = 0.0;
    heading_hold_feedback_rps_ = 0.0;
    heading_hold_feedforward_rps_ = 0.0;
    heading_hold_correction_rps_ = 0.0;
    heading_hold_active_kp_ = 0.0;
    heading_hold_active_kd_ = 0.0;
    heading_hold_effective_limit_rps_ = 0.0;
  }

  double controlled_angular_velocity()
  {
    const bool linear_motion =
      std::hypot(commanded_vx_mps_, commanded_vy_mps_) >=
      heading_hold_linear_deadband_mps_;
    const bool intentional_rotation =
      std::abs(commanded_wz_rps_) >= heading_hold_angular_deadband_rps_;

    if (!heading_hold_enabled_ || !linear_motion || intentional_rotation) {
      reset_heading_hold();
      return commanded_wz_rps_;
    }

    const bool imu_fresh = have_latest_imu_ &&
      (steady_clock_.now() - latest_imu_time_).seconds() <= heading_hold_imu_timeout_s_;
    if (!imu_fresh) {
      reset_heading_hold();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Heading hold unavailable: waiting for a fresh BNO055 sample");
      return commanded_wz_rps_;
    }

    if (!heading_hold_active_) {
      heading_hold_target_rad_ = latest_yaw_rad_;
      heading_hold_active_ = true;
      ++heading_hold_activations_;
    }

    heading_hold_error_rad_ = normalize_angle(
      heading_hold_target_rad_ - latest_yaw_rad_);

    const double absolute_x = std::abs(commanded_vx_mps_);
    const double absolute_y = std::abs(commanded_vy_mps_);
    const double linear_sum = absolute_x + absolute_y;
    const double lateral_weight = linear_sum > 0.0 ? absolute_y / linear_sum : 0.0;
    const auto blend = [lateral_weight](double longitudinal, double lateral) {
        return longitudinal + lateral_weight * (lateral - longitudinal);
      };

    heading_hold_active_kp_ = blend(heading_hold_kp_, heading_hold_lateral_kp_);
    heading_hold_active_kd_ = blend(heading_hold_kd_, heading_hold_lateral_kd_);
    const double configured_limit = blend(
      heading_hold_max_correction_rps_, heading_hold_lateral_max_correction_rps_);

    if (lateral_weight >= 0.75) {
      heading_hold_mode_ = "lateral";
    } else if (lateral_weight <= 0.25) {
      heading_hold_mode_ = "longitudinal";
    } else {
      heading_hold_mode_ = "diagonal";
    }

    const double error_deadband_rad = heading_hold_error_deadband_deg_ * kPi / 180.0;
    const double proportional_error =
      std::abs(heading_hold_error_rad_) < error_deadband_rad ?
      0.0 : heading_hold_error_rad_;
    heading_hold_feedback_rps_ =
      heading_hold_active_kp_ * proportional_error -
      heading_hold_active_kd_ * latest_gyro_z_rad_s_;

    heading_hold_feedforward_rps_ = 0.0;
    if (absolute_y >= heading_hold_linear_deadband_mps_) {
      heading_hold_feedforward_rps_ =
        -std::copysign(heading_hold_lateral_feedforward_rps_, commanded_vy_mps_) *
        lateral_weight;
    }

    const double mecanum_radius_m = 0.5 * (robot_length_m_ + robot_width_m_);
    const double linear_authority_limit = mecanum_radius_m > 0.0 ?
      heading_hold_linear_authority_ratio_ *
      std::hypot(commanded_vx_mps_, commanded_vy_mps_) / mecanum_radius_m :
      configured_limit;
    heading_hold_effective_limit_rps_ = std::min(
      configured_limit, linear_authority_limit);
    heading_hold_correction_rps_ = std::clamp(
      heading_hold_feedback_rps_ + heading_hold_feedforward_rps_,
      -heading_hold_effective_limit_rps_, heading_hold_effective_limit_rps_);
    return heading_hold_correction_rps_;
  }

  std::array<int16_t, 4> calculate_motor_pwm(double angular_z_rps) const
  {
    const double k = 0.5 * (robot_length_m_ + robot_width_m_);
    std::array<double, 4> wheel_speed{{
      commanded_vx_mps_ - commanded_vy_mps_ - k * angular_z_rps,
      commanded_vx_mps_ + commanded_vy_mps_ + k * angular_z_rps,
      commanded_vx_mps_ + commanded_vy_mps_ - k * angular_z_rps,
      commanded_vx_mps_ - commanded_vy_mps_ + k * angular_z_rps
    }};

    double greatest_speed = 0.0;
    for (const double speed : wheel_speed) {
      greatest_speed = std::max(greatest_speed, std::abs(speed));
    }
    if (greatest_speed > maximum_wheel_speed_mps_) {
      const double scale = maximum_wheel_speed_mps_ / greatest_speed;
      for (double & speed : wheel_speed) {
        speed *= scale;
      }
    }

    const bool rotation_only =
      std::hypot(commanded_vx_mps_, commanded_vy_mps_) < velocity_deadband_mps_ &&
      std::abs(angular_z_rps) > 0.0;
    const int effective_minimum_pwm =
      rotation_only ? minimum_rotation_pwm_ : minimum_pwm_;

    std::array<int16_t, 4> pwm{{0, 0, 0, 0}};
    for (std::size_t index = 0; index < pwm.size(); ++index) {
      pwm[index] = wheel_speed_to_pwm(wheel_speed[index], effective_minimum_pwm);
    }
    return pwm;
  }

  std::vector<uint8_t> make_motor_command_frame(
    const std::array<int16_t, 4> & pwm, uint8_t flags)
  {
    constexpr uint16_t payload_size = 11;
    std::vector<uint8_t> frame;
    frame.reserve(2U + 4U + payload_size + 2U);
    frame.push_back(kSof1);
    frame.push_back(kSof2);
    frame.push_back(kProtocolVersion);
    frame.push_back(kTypeMecanumCommand);
    append_u16_le(frame, payload_size);
    append_u16_le(frame, command_sequence_++);
    for (const int16_t value : pwm) {
      append_i16_le(frame, value);
    }
    frame.push_back(flags);

    const uint16_t crc = crc16_ccitt(frame.data() + 2, frame.size() - 2U);
    append_u16_le(frame, crc);
    return frame;
  }

  void queue_motor_command(const std::array<int16_t, 4> & pwm, uint8_t flags)
  {
    if (serial_fd_ < 0 || steady_clock_.now() < serial_command_ready_time_) {
      return;
    }

    // Nunca interrompe um quadro parcialmente escrito. Antes do primeiro byte,
    // substitui o quadro pendente pelo comando mais recente.
    if (pending_tx_offset_ == 0U) {
      pending_tx_frame_ = make_motor_command_frame(pwm, flags);
    }
  }

  void flush_serial_tx()
  {
    if (serial_fd_ < 0 || pending_tx_frame_.empty()) {
      return;
    }

    const uint8_t * data = pending_tx_frame_.data() + pending_tx_offset_;
    const std::size_t remaining = pending_tx_frame_.size() - pending_tx_offset_;
    const ssize_t written = write(serial_fd_, data, remaining);
    if (written > 0) {
      host_tx_bytes_ += static_cast<uint64_t>(written);
      pending_tx_offset_ += static_cast<std::size_t>(written);
      if (pending_tx_offset_ == pending_tx_frame_.size()) {
        pending_tx_frame_.clear();
        pending_tx_offset_ = 0;
      }
    } else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      handle_serial_error("write");
    }
  }

  void send_periodic_command()
  {
    if (emergency_stop_) {
      reset_heading_hold();
      queue_motor_command({0, 0, 0, 0}, kFlagEstop);
      return;
    }

    if (!have_velocity_command_) {
      reset_heading_hold();
      queue_motor_command({0, 0, 0, 0}, 0);
      return;
    }

    const double command_age = (steady_clock_.now() - last_velocity_command_time_).seconds();
    if (command_age > command_timeout_s_) {
      reset_heading_hold();
      queue_motor_command({0, 0, 0, 0}, 0);
      return;
    }

    queue_motor_command(calculate_motor_pwm(controlled_angular_velocity()), kFlagEnable);
  }

  void send_emergency_stop_best_effort()
  {
    if (serial_fd_ < 0 || steady_clock_.now() < serial_command_ready_time_) {
      return;
    }
    const std::vector<uint8_t> frame =
      make_motor_command_frame({0, 0, 0, 0}, kFlagEstop);
    [[maybe_unused]] const ssize_t written =
      write(serial_fd_, frame.data(), frame.size());
  }

  void monitor_connection()
  {
    if (serial_fd_ < 0) {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus item;
      item.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      item.name = "robo_hardware/mega_delta2d_bno055";
      item.hardware_id = "arduino_mega_2560_ch340";
      item.message = "Serial disconnected";
      item.values.push_back(key_value("serial_port", serial_port_));
      array.status.push_back(std::move(item));
      diagnostics_publisher_->publish(array);
      return;
    }

    const rclcpp::Time steady_now = steady_clock_.now();
    if (steady_now < serial_command_ready_time_) {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus item;
      item.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      item.name = "robo_hardware/mega_delta2d_bno055";
      item.hardware_id = "arduino_mega_2560_ch340";
      item.message = "Serial open; respecting Mega startup quiet period";
      item.values.push_back(key_value("serial_port", serial_port_));
      item.values.push_back(key_value(
        "startup_remaining_s",
        std::to_string((serial_command_ready_time_ - steady_now).seconds())));
      item.values.push_back(key_value("host_rx_bytes", std::to_string(host_rx_bytes_)));
      item.values.push_back(key_value("host_tx_bytes", std::to_string(host_tx_bytes_)));
      array.status.push_back(std::move(item));
      diagnostics_publisher_->publish(array);
      return;
    }

    if (!have_mega_status_) {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus item;
      item.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      item.name = "robo_hardware/mega_delta2d_bno055";
      item.hardware_id = "arduino_mega_2560_ch340";
      item.message = "Serial open; awaiting Mega telemetry";
      item.values.push_back(key_value("serial_port", serial_port_));
      item.values.push_back(key_value("host_rx_bytes", std::to_string(host_rx_bytes_)));
      item.values.push_back(key_value("host_rx_reads", std::to_string(host_rx_reads_)));
      item.values.push_back(key_value("host_tx_bytes", std::to_string(host_tx_bytes_)));
      array.status.push_back(std::move(item));
      diagnostics_publisher_->publish(array);
      return;
    }

    if ((steady_clock_.now() - last_status_time_).seconds() > 2.0) {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus item;
      item.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      item.name = "robo_hardware/mega_delta2d_bno055";
      item.hardware_id = "arduino_mega_2560_ch340";
      item.message = "Serial open, but Mega telemetry is stale";
      item.values.push_back(key_value("serial_port", serial_port_));
      item.values.push_back(key_value("host_rx_bytes", std::to_string(host_rx_bytes_)));
      item.values.push_back(key_value("host_rx_reads", std::to_string(host_rx_reads_)));
      item.values.push_back(key_value("host_tx_bytes", std::to_string(host_tx_bytes_)));
      item.values.push_back(key_value("published_scans", std::to_string(published_scans_)));
      array.status.push_back(std::move(item));
      diagnostics_publisher_->publish(array);
    }
  }

  std::string serial_port_;
  int serial_baud_{500000};
  double serial_startup_delay_s_{5.0};
  double robot_length_m_{0.340};
  double robot_width_m_{0.615};
  double maximum_wheel_speed_mps_{0.450};
  double velocity_deadband_mps_{0.005};
  int minimum_pwm_{150};
  int minimum_rotation_pwm_{100};
  int maximum_pwm_{215};
  double command_rate_hz_{20.0};
  double command_timeout_s_{0.25};
  std::string scan_frame_{"laser_raw"};
  double scan_range_min_m_{0.08};
  double scan_range_max_m_{6.0};
  std::string imu_frame_{"imu_link"};
  double imu_yaw_sign_{-1.0};
  double imu_yaw_offset_deg_{0.0};
  bool zero_imu_on_start_{true};
  bool have_yaw_reference_{false};
  double yaw_reference_rad_{0.0};
  bool heading_hold_enabled_{true};
  double heading_hold_kp_{0.8};
  double heading_hold_kd_{0.05};
  double heading_hold_max_correction_rps_{0.15};
  double heading_hold_lateral_kp_{1.2};
  double heading_hold_lateral_kd_{0.08};
  double heading_hold_lateral_max_correction_rps_{0.15};
  double heading_hold_lateral_feedforward_rps_{0.025};
  double heading_hold_error_deadband_deg_{0.5};
  double heading_hold_linear_authority_ratio_{0.75};
  double heading_hold_linear_deadband_mps_{0.01};
  double heading_hold_angular_deadband_rps_{0.02};
  double heading_hold_imu_timeout_s_{0.20};
  bool have_latest_imu_{false};
  bool heading_hold_active_{false};
  double latest_yaw_rad_{0.0};
  double latest_gyro_z_rad_s_{0.0};
  double heading_hold_target_rad_{0.0};
  double heading_hold_error_rad_{0.0};
  std::string heading_hold_mode_{"inactive"};
  double heading_hold_feedback_rps_{0.0};
  double heading_hold_feedforward_rps_{0.0};
  double heading_hold_correction_rps_{0.0};
  double heading_hold_active_kp_{0.0};
  double heading_hold_active_kd_{0.0};
  double heading_hold_effective_limit_rps_{0.0};
  uint64_t heading_hold_activations_{0};
  rclcpp::Time latest_imu_time_{0, 0, RCL_STEADY_TIME};

  int serial_fd_{-1};
  rclcpp::Clock steady_clock_;
  rclcpp::Time next_reconnect_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time serial_open_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time serial_command_ready_time_{0, 0, RCL_STEADY_TIME};

  bool rx_saw_first_sof_{false};
  bool rx_receiving_{false};
  std::size_t rx_expected_size_{0};
  std::vector<uint8_t> rx_frame_;

  std::vector<uint8_t> pending_tx_frame_;
  std::size_t pending_tx_offset_{0};
  uint16_t command_sequence_{0};

  double commanded_vx_mps_{0.0};
  double commanded_vy_mps_{0.0};
  double commanded_wz_rps_{0.0};
  bool have_velocity_command_{false};
  bool emergency_stop_{false};
  rclcpp::Time last_velocity_command_time_{0, 0, RCL_STEADY_TIME};

  MegaStatus last_mega_status_{};
  bool have_mega_status_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_STEADY_TIME};

  uint64_t host_bad_crc_frames_{0};
  uint64_t host_invalid_frames_{0};
  uint64_t published_scans_{0};
  uint64_t published_imu_{0};
  uint64_t host_rx_bytes_{0};
  uint64_t host_rx_reads_{0};
  uint64_t host_tx_bytes_{0};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
