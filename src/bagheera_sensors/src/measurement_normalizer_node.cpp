// Normalize MowgliNext measurements for standard ROS consumers: fill
// required frames, valid quaternions and non-zero covariances.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "bagheera_sensors/sensor_math.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace bagheera_sensors
{

namespace
{
template<typename Array>
bool all_zero(const Array & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return value == 0.0;});
}

template<typename Quaternion>
bool normalize_quaternion(Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm <= 1.0e-12 || !std::isfinite(norm)) {
    quaternion.x = quaternion.y = quaternion.z = 0.0;
    quaternion.w = 1.0;
    return false;
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return true;
}
}  // namespace

class MeasurementNormalizerNode : public rclcpp::Node
{
public:
  explicit MeasurementNormalizerNode(const rclcpp::NodeOptions & options)
  : Node("bagheera_measurement_normalizer", options)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    axle_to_base_link_m_ = declare_parameter<double>("axle_to_base_link_m", 0.0);
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    const bool imu_enabled = declare_parameter<bool>("imu_enabled", true);
    camera_frame_ = declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");
    camera_width_ = static_cast<uint32_t>(declare_parameter<int64_t>("camera_width", 1920));
    camera_height_ = static_cast<uint32_t>(declare_parameter<int64_t>("camera_height", 1080));
    fx_ = declare_parameter<double>("camera_nominal_fx", 960.0);
    fy_ = declare_parameter<double>("camera_nominal_fy", 960.0);
    cx_ = declare_parameter<double>("camera_nominal_cx", 960.0);
    cy_ = declare_parameter<double>("camera_nominal_cy", 540.0);
    wheel_pose_covariance_ = diagonal6(read_variances(
        "wheel_pose_variance", {0.04, 0.09, 1.0e6, 1.0e6, 1.0e6, 0.09}, 6));
    wheel_twist_covariance_ = diagonal6(read_variances(
        "wheel_twist_variance", {0.01, 0.01, 1.0e6, 1.0e6, 1.0e6, 0.0009}, 6));
    imu_orientation_covariance_ = diagonal3(read_variances(
        "imu_orientation_variance", {0.0027, 0.0027, 0.01}, 3));
    imu_angular_velocity_covariance_ = diagonal3(read_variances(
        "imu_angular_velocity_variance", {0.0004, 0.0004, 0.0004}, 3));
    imu_linear_acceleration_covariance_ = diagonal3(read_variances(
        "imu_linear_acceleration_variance", {0.04, 0.04, 0.04}, 3));
    if (!std::isfinite(axle_to_base_link_m_)) {
      throw std::invalid_argument("axle_to_base_link_m must be finite");
    }
    if (camera_width_ == 0 || camera_height_ == 0) {
      throw std::invalid_argument("camera_width and camera_height must be positive");
    }
    if (fx_ <= 0.0 || fy_ <= 0.0) {
      throw std::invalid_argument("camera_nominal_fx and camera_nominal_fy must be positive");
    }

    wheel_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 20);
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom_raw", 20,
      [this](nav_msgs::msg::Odometry::UniquePtr message) {normalize_wheel(std::move(message));});
    // The EKF fuses only the WT901; the mainboard IMU is off by default.
    if (imu_enabled) {
      imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 20);
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", 20,
        [this](sensor_msgs::msg::Imu::UniquePtr message) {normalize_imu(std::move(message));});
    }
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info_raw", 10,
      [this](sensor_msgs::msg::CameraInfo::UniquePtr message) {
        normalize_camera_info(std::move(message));
      });
  }

private:
  std::vector<double> read_variances(
    const std::string & name, const std::vector<double> & defaults, std::size_t length)
  {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != length ||
      std::any_of(
        values.begin(), values.end(),
        [](double value) {return !std::isfinite(value) || value < 0.0;}))
    {
      throw std::invalid_argument(
              name + " must contain " + std::to_string(length) + " finite non-negative values");
    }
    return values;
  }

  static std::array<double, 36> diagonal6(const std::vector<double> & diagonal)
  {
    std::array<double, 36> covariance{};
    for (std::size_t index = 0; index < 6; ++index) {
      covariance[index * 6 + index] = diagonal[index];
    }
    return covariance;
  }

  static std::array<double, 9> diagonal3(const std::vector<double> & diagonal)
  {
    std::array<double, 9> covariance{};
    for (std::size_t index = 0; index < 3; ++index) {
      covariance[index * 3 + index] = diagonal[index];
    }
    return covariance;
  }

  void ensure_stamp(builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = now();
    }
  }

  void normalize_wheel(nav_msgs::msg::Odometry::UniquePtr message)
  {
    ensure_stamp(message->header.stamp);
    if (message->header.frame_id.empty()) {
      message->header.frame_id = odom_frame_;
    }
    if (message->child_frame_id.empty()) {
      message->child_frame_id = base_frame_;
    }
    normalize_quaternion(message->pose.pose.orientation);
    if (all_zero(message->pose.covariance)) {
      message->pose.covariance = wheel_pose_covariance_;
    }
    if (all_zero(message->twist.covariance)) {
      message->twist.covariance = wheel_twist_covariance_;
    }
    if (axle_to_base_link_m_ != 0.0) {
      auto & twist = message->twist.twist;
      const double axle_vy = twist.linear.y;
      const auto [base_vx, offset_vy] =
        axle_to_base_link_twist(twist.linear.x, twist.angular.z, axle_to_base_link_m_);
      twist.linear.x = base_vx;
      twist.linear.y = axle_vy + offset_vy;
      message->twist.covariance =
        shift_twist_covariance_x(message->twist.covariance, axle_to_base_link_m_);
    }
    wheel_publisher_->publish(std::move(message));
  }

  void normalize_imu(sensor_msgs::msg::Imu::UniquePtr message)
  {
    ensure_stamp(message->header.stamp);
    if (message->header.frame_id.empty()) {
      message->header.frame_id = imu_frame_;
    }
    if (!normalize_quaternion(message->orientation)) {
      message->orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    } else if (all_zero(message->orientation_covariance)) {
      message->orientation_covariance = imu_orientation_covariance_;
    }
    if (all_zero(message->angular_velocity_covariance)) {
      message->angular_velocity_covariance = imu_angular_velocity_covariance_;
    }
    if (all_zero(message->linear_acceleration_covariance)) {
      message->linear_acceleration_covariance = imu_linear_acceleration_covariance_;
    }
    imu_publisher_->publish(std::move(message));
  }

  // Fill required image dimensions while preserving real calibration data.
  // camera_ros publishes an otherwise valid uncalibrated CameraInfo with
  // width and height zero when no calibration file is installed. Foxglove
  // needs those dimensions and non-zero focal lengths to display the image.
  // The nominal pinhole intrinsics are for visualization only; metric vision
  // must use an actual fisheye calibration.
  void normalize_camera_info(sensor_msgs::msg::CameraInfo::UniquePtr message)
  {
    ensure_stamp(message->header.stamp);
    if (message->header.frame_id.empty()) {
      message->header.frame_id = camera_frame_;
    }
    if (message->width == 0) {
      message->width = camera_width_;
    }
    if (message->height == 0) {
      message->height = camera_height_;
    }
    if (message->k[0] == 0.0 || message->k[4] == 0.0) {
      message->distortion_model = "plumb_bob";
      message->d = std::vector<double>(5, 0.0);
      message->k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
      message->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      message->p = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    }
    camera_info_publisher_->publish(std::move(message));
  }

  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string camera_frame_;
  double axle_to_base_link_m_{};
  uint32_t camera_width_{};
  uint32_t camera_height_{};
  double fx_{};
  double fy_{};
  double cx_{};
  double cy_{};
  std::array<double, 36> wheel_pose_covariance_{};
  std::array<double, 36> wheel_twist_covariance_{};
  std::array<double, 9> imu_orientation_covariance_{};
  std::array<double, 9> imu_angular_velocity_covariance_{};
  std::array<double, 9> imu_linear_acceleration_covariance_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
};

}  // namespace bagheera_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(bagheera_sensors::MeasurementNormalizerNode)
