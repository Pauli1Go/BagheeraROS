// ROS 2 driver for a WIT Motion WT901 connected to Raspberry Pi I2C.

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "bagheera_sensors/sensor_math.hpp"
#include "bagheera_sensors/wt901_i2c.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/string.hpp"

namespace bagheera_sensors
{

namespace
{
std::array<double, 9> diagonal_covariance(double value)
{
  return {value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value};
}
}  // namespace

// Poll the WT901 registers and publish un-oriented SI-unit IMU data.
class Wt901Node : public rclcpp::Node
{
public:
  explicit Wt901Node(const rclcpp::NodeOptions & options)
  : Node("bagheera_wt901", options)
  {
    const int bus = static_cast<int>(declare_parameter<int64_t>("i2c_bus", 1));
    const int address = static_cast<int>(declare_parameter<int64_t>("i2c_address", 0x50));
    const double rate = declare_parameter<double>("publish_rate", 50.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    calibration_samples_ =
      static_cast<std::size_t>(declare_parameter<int64_t>("gyro_calibration_samples", 200));
    const double angular_variance = declare_parameter<double>("angular_velocity_variance", 0.0001);
    const double acceleration_variance =
      declare_parameter<double>("linear_acceleration_variance", 0.04);
    const double magnetic_variance = declare_parameter<double>("magnetic_field_variance", 2.5e-11);
    const int configured_mag_type =
      static_cast<int>(declare_parameter<int64_t>("mag_sensor_type", -1));
    publish_magnetometer_ = declare_parameter<bool>("publish_magnetometer", true);

    if (rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    if (angular_variance <= 0.0 || acceleration_variance <= 0.0 || magnetic_variance <= 0.0) {
      throw std::invalid_argument("IMU variances must be positive");
    }
    block_length_ = publish_magnetometer_ ? kWt901MotionBlockLength : kWt901InertialBlockLength;

    bus_ = std::make_unique<Wt901I2c>(bus, address);
    if (publish_magnetometer_) {
      int mag_type = configured_mag_type;
      if (mag_type < 0) {
        const auto data = bus_->read_block(kWt901MagSensorRegister, 2);
        mag_type = little_endian_int16(data[0], data[1]);
      }
      // Validate the reported sensor type before starting the timer.
      magnetic_scale_ = magnetic_scale_tesla(mag_type);
      mag_sensor_type_ = mag_type;
    }
    bias_ = std::make_unique<GyroBiasEstimator>(calibration_samples_);
    angular_covariance_ = diagonal_covariance(angular_variance);
    acceleration_covariance_ = diagonal_covariance(acceleration_variance);
    magnetic_covariance_ = diagonal_covariance(magnetic_variance);
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/wt901/data_raw", rclcpp::SensorDataQoS());
    if (publish_magnetometer_) {
      mag_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>(
        "/imu/wt901/mag_raw", rclcpp::SensorDataQoS());
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {poll();});

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    sleep_subscription_ = create_subscription<std_msgs::msg::String>(
      "/dock/sleep_state", latched,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_sleep_state(message->data);});

    const std::string magnetometer = publish_magnetometer_ ?
      "enabled (type " + std::to_string(*mag_sensor_type_) + ")" : "disabled";
    RCLCPP_INFO(
      get_logger(), "WT901 on /dev/i2c-%d address 0x%02x; magnetometer %s; "
      "keep robot still for %.1f s", bus, address, magnetometer.c_str(),
      static_cast<double>(calibration_samples_) / rate);
  }

private:
  void on_sleep_state(const std::string & state)
  {
    const bool sleeping = state == "sleeping";
    if (sleeping == sleeping_) {
      return;
    }
    sleeping_ = sleeping;
    if (sleeping) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "WT901 polling stopped while docked");
      return;
    }
    // The robot still stands in the dock: measure the gyro bias again.
    // Nothing is published until that is done, which dock_sleep waits for.
    bias_ = std::make_unique<GyroBiasEstimator>(calibration_samples_);
    calibration_announced_ = false;
    timer_->reset();
    RCLCPP_INFO(get_logger(), "WT901 polling resumed; re-measuring gyro bias");
  }

  void poll()
  {
    ImuSample sample{};
    try {
      const auto data = bus_->read_block(kWt901MotionRegister, block_length_);
      sample = decode_wt901_block(data.data(), data.size());
      consecutive_errors_ = 0;
    } catch (const std::exception & error) {
      ++consecutive_errors_;
      if (consecutive_errors_ == 1 || consecutive_errors_ % 50 == 0) {
        RCLCPP_ERROR(
          get_logger(), "WT901 I2C read failed (%zu): %s", consecutive_errors_, error.what());
      }
      return;
    }

    const auto stamp = now();
    if (mag_publisher_) {
      sensor_msgs::msg::MagneticField magnetic;
      magnetic.header.stamp = stamp;
      magnetic.header.frame_id = frame_id_;
      magnetic.magnetic_field.x = sample.magnetic_raw[0] * magnetic_scale_;
      magnetic.magnetic_field.y = sample.magnetic_raw[1] * magnetic_scale_;
      magnetic.magnetic_field.z = sample.magnetic_raw[2] * magnetic_scale_;
      magnetic.magnetic_field_covariance = magnetic_covariance_;
      mag_publisher_->publish(magnetic);
    }

    if (!bias_->update(sample.angular_velocity)) {
      return;
    }
    if (!calibration_announced_) {
      const auto & bias = bias_->bias();
      RCLCPP_INFO(
        get_logger(), "WT901 gyro bias ready: [%.6f, %.6f, %.6f] rad/s",
        bias[0], bias[1], bias[2]);
      calibration_announced_ = true;
    }

    const auto angular_velocity = bias_->correct(sample.angular_velocity);
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    // The WT901 orientation may use magnetometer yaw. Leave it explicitly
    // unavailable; robot_localization consumes only angular_velocity.z.
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity.x = angular_velocity[0];
    message.angular_velocity.y = angular_velocity[1];
    message.angular_velocity.z = angular_velocity[2];
    message.angular_velocity_covariance = angular_covariance_;
    message.linear_acceleration.x = sample.acceleration[0];
    message.linear_acceleration.y = sample.acceleration[1];
    message.linear_acceleration.z = sample.acceleration[2];
    message.linear_acceleration_covariance = acceleration_covariance_;
    publisher_->publish(message);
  }

  std::unique_ptr<Wt901I2c> bus_;
  std::unique_ptr<GyroBiasEstimator> bias_;
  std::string frame_id_;
  std::size_t calibration_samples_{};
  std::size_t block_length_{};
  bool publish_magnetometer_{};
  std::optional<int> mag_sensor_type_;
  double magnetic_scale_{};
  std::array<double, 9> angular_covariance_{};
  std::array<double, 9> acceleration_covariance_{};
  std::array<double, 9> magnetic_covariance_{};
  std::size_t consecutive_errors_ = 0;
  bool calibration_announced_ = false;
  bool sleeping_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sleep_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace bagheera_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(bagheera_sensors::Wt901Node)
