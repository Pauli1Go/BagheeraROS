// ROS 2 publisher for the downward-facing Bagheera PMW3901.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "bagheera_sensors/pmw3901.hpp"
#include "bagheera_sensors/sensor_math.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace bagheera_sensors
{

class OpticalFlowNode : public rclcpp::Node
{
public:
  explicit OpticalFlowNode(const rclcpp::NodeOptions & options)
  : Node("bagheera_optical_flow", options)
  {
    const int bus = static_cast<int>(declare_parameter<int64_t>("spi_bus", 0));
    const int chip_select = static_cast<int>(declare_parameter<int64_t>("spi_chip_select", 0));
    const auto speed_hz = static_cast<uint32_t>(declare_parameter<int64_t>("spi_speed_hz", 400000));
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    const double rate = declare_parameter<double>("publish_rate", 100.0);
    meters_per_count_ = declare_parameter<double>("mount_height_m", 0.09) *
      declare_parameter<double>("radians_per_count", 0.002057);
    swap_xy_ = declare_parameter<bool>("swap_xy", false);
    invert_x_ = declare_parameter<bool>("invert_x", false);
    invert_y_ = declare_parameter<bool>("invert_y", false);
    minimum_quality_ = static_cast<int>(declare_parameter<int64_t>("minimum_quality", 0));
    // Lever arm of the sensor relative to the base_link rotation centre.
    // While rotating, the sensor sweeps with wz x r, which would otherwise be
    // fused as a phantom base velocity (vy) and smear the odometry.
    lever_x_ = declare_parameter<double>("lever_arm_x", 0.0);
    lever_y_ = declare_parameter<double>("lever_arm_y", 0.0);
    const std::string imu_topic =
      declare_parameter<std::string>("imu_topic", "/imu/wt901/data_raw");
    imu_max_age_s_ = declare_parameter<double>("imu_max_age", 0.5);
    // The PMW3901 reports phantom translation while the robot pivots. Such
    // readings get an inflated covariance so the EKF's Mahalanobis gate drops
    // them instead of integrating the phantom.
    rotation_gate_ = declare_parameter<double>("rotation_gate_rad_s", 0.25);
    rotation_variance_ = declare_parameter<double>("rotation_variance", 1000000.0);
    rotation_gate_enabled_ = declare_parameter<bool>("rotation_gate_enabled", true);
    if (rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    imu_max_age_ns_ = static_cast<int64_t>(imu_max_age_s_ * 1e9);

    rclcpp::QoS sensor_qos(20);
    sensor_qos.best_effort();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        gyro_.add(now().nanoseconds(), message->angular_velocity.z);
      });

    sensor_ = std::make_unique<Pmw3901>(bus, chip_select, speed_hz);
    const auto [product, revision] = sensor_->identity();
    raw_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/optical_flow/raw", 20);
    twist_publisher_ =
      create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/optical_flow/twist", 20);
    last_stamp_ns_ = now().nanoseconds();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {poll();});

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    sleep_subscription_ = create_subscription<std_msgs::msg::String>(
      "/dock/sleep_state", latched,
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_sleep_state(message->data);});

    RCLCPP_INFO(
      get_logger(), "PMW3901 ready on SPI%d.%d (id=0x%02x revision=0x%02x, %.6f m/count)",
      bus, chip_select, product, revision, meters_per_count_);
  }

private:
  void on_sleep_state(const std::string & state)
  {
    const bool sleeping = state == "sleeping";
    if (sleeping == sleeping_) {
      return;
    }
    if (sleeping) {
      timer_->cancel();
      try {
        sensor_->shutdown();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "PMW3901 shutdown failed: %s", error.what());
      }
      sleeping_ = true;
      RCLCPP_INFO(get_logger(), "PMW3901 asleep (LED off) while docked");
      return;
    }
    try {
      sensor_->power_up();
      // Drop whatever the first burst after the reset reports.
      sensor_->read_motion();
    } catch (const std::exception & error) {
      // Stay asleep; the next state message retries.
      RCLCPP_ERROR(get_logger(), "PMW3901 wake-up failed: %s", error.what());
      return;
    }
    sleeping_ = false;
    last_stamp_ns_ = now().nanoseconds();
    timer_->reset();
    RCLCPP_INFO(get_logger(), "PMW3901 awake");
  }

  void poll()
  {
    FlowSample motion{};
    try {
      motion = sensor_->read_motion();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "PMW3901 SPI read failed: %s", error.what());
      return;
    }
    const rclcpp::Time stamp_time = now();
    const int64_t now_ns = stamp_time.nanoseconds();
    const double elapsed = static_cast<double>(now_ns - last_stamp_ns_) / 1e9;
    last_stamp_ns_ = now_ns;
    const auto stamp = static_cast<builtin_interfaces::msg::Time>(stamp_time);

    if (raw_publisher_->get_subscription_count() > 0) {
      geometry_msgs::msg::Vector3Stamped raw;
      raw.header.stamp = stamp;
      raw.header.frame_id = "pmw3901_sensor";
      raw.vector.x = motion.delta_x;
      raw.vector.y = motion.delta_y;
      raw.vector.z = motion.quality;
      raw_publisher_->publish(raw);
    }

    if (motion.quality < minimum_quality_ || elapsed <= 0.0 || !std::isfinite(elapsed)) {
      return;
    }
    const auto [robot_x, robot_y] =
      transform_counts(motion.delta_x, motion.delta_y, swap_xy_, invert_x_, invert_y_);
    double vx = robot_x * meters_per_count_ / elapsed;
    double vy = robot_y * meters_per_count_ / elapsed;

    // Remove the rotation-induced sweep velocity so the twist describes the
    // base_link motion, not the sensor point motion.
    std::optional<double> wz;
    if (lever_x_ != 0.0 || lever_y_ != 0.0) {
      wz = gyro_.mean_between(now_ns - static_cast<int64_t>(elapsed * 1e9), now_ns);
      if (!wz) {
        wz = gyro_.latest_within(now_ns, imu_max_age_ns_);
      }
      if (!wz) {
        // Normal for 4 s while the WT901 measures its bias.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No gyro data within %.1f s; publishing uncorrected flow", imu_max_age_s_);
      } else {
        // v_flow = v_base + wz x lever_arm  =>  v_base = v_flow - wz x r
        vx += *wz * lever_y_;
        vy -= *wz * lever_x_;
      }
    }

    double variance = std::max(0.0025, 0.25 / std::max(1, motion.quality));
    if (!wz) {
      wz = gyro_.mean_between(now_ns - 300000000LL, now_ns);
    }
    if (rotation_gate_enabled_ && wz && std::abs(*wz) > rotation_gate_) {
      variance = rotation_variance_;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = frame_id_;
    twist.twist.twist.linear.x = vx;
    twist.twist.twist.linear.y = vy;
    twist.twist.covariance[0] = variance;
    twist.twist.covariance[7] = variance;
    twist.twist.covariance[14] = 1000000.0;
    twist.twist.covariance[21] = 1000000.0;
    twist.twist.covariance[28] = 1000000.0;
    twist.twist.covariance[35] = 1000000.0;
    twist_publisher_->publish(twist);
  }

  std::unique_ptr<Pmw3901> sensor_;
  std::string frame_id_;
  double meters_per_count_{};
  bool swap_xy_{};
  bool invert_x_{};
  bool invert_y_{};
  int minimum_quality_{};
  double lever_x_{};
  double lever_y_{};
  double imu_max_age_s_{};
  int64_t imu_max_age_ns_{};
  double rotation_gate_{};
  double rotation_variance_{};
  bool rotation_gate_enabled_{};
  bool sleeping_ = false;
  int64_t last_stamp_ns_{};
  GyroHistory gyro_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sleep_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr raw_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace bagheera_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(bagheera_sensors::OpticalFlowNode)
