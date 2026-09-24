// Copyright 2026 Paul Praschl
// Licensed under the Apache License, Version 2.0.

#ifndef BAGHEERA_DOCKING__TAG_CHARGING_DOCK_HPP_
#define BAGHEERA_DOCKING__TAG_CHARGING_DOCK_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace bagheera_docking
{

/**
 * Charging dock for Bagheera's two AprilTags.
 *
 * ID 1 sits on the dock and supplies the dock position. ID 0 hangs on the
 * wall above it and supplies the dock-axis angle; the small ID 1 is far too
 * noisy for a plane normal at the staging distance. Both detections arrive
 * in the camera frame with their image stamp and are fused in the docking
 * server's fixed frame (odom), where the dock does not move. Once ID 0 leaves
 * the image near the dock, its filtered angle is held. When ID 1 disappears
 * in the last centimetres, the last refined pose is held inside
 * hold_distance.
 *
 * Nav2's graceful controller only finishes its curve at its target, so the
 * server is given a pre-dock pose pre_dock_distance in front of the contact
 * pose on the same axis. Reaching it counts as "docked" for the server; the
 * straight final approach is driven by bagheera_dock_trigger during the
 * wait-for-charge phase. /dock_pose publishes the real contact pose.
 *
 * Contact is the raw charging voltage; charging is the debounced /docked.
 */
class TagChargingDock : public opennav_docking_core::ChargingDock
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) override;
  bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id) override;
  bool isDocked() override;
  bool isCharging() override;
  bool disableCharging() override;
  bool hasStoppedCharging() override;

private:
  // Transform a stamped camera-frame detection into the fixed frame at its
  // image time. Returns false if TF cannot provide that transform.
  bool toFixedFrame(
    const geometry_msgs::msg::PoseStamped & in, const std::string & frame,
    geometry_msgs::msg::PoseStamped & out);
  void updateAxis(const std::string & frame);
  bool updatePosition(const std::string & frame);
  bool robotPose(const std::string & frame, geometry_msgs::msg::PoseStamped & out);
  geometry_msgs::msg::PoseStamped composeDockPose() const;
  void logDiagnostics(const std::string & frame);
  void resetRun();
  void publishEvent(const std::string & text);
  bool runActive() const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("TagChargingDock")};
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr axis_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr power_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docked_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    dock_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    staging_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr event_pub_;

  mutable std::mutex mutex_;
  geometry_msgs::msg::PoseStamped latest_position_;
  geometry_msgs::msg::PoseStamped latest_axis_;
  double charge_voltage_{0.0};
  bool docked_{false};

  // Per-run fused state in the fixed frame.
  rclcpp::Time last_position_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_axis_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_refine_call_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diagnostic_{0, 0, RCL_ROS_TIME};
  std::optional<double> tag_x_;
  std::optional<double> tag_y_;
  std::optional<double> axis_yaw_;
  std::string fused_frame_;
  bool holding_{false};
  // Set by isDocked() when the pre-dock line is reached off axis; the next
  // getRefinedPose() then fails so that Nav2 retries at once.
  bool pre_dock_rejected_{false};
  geometry_msgs::msg::PoseStamped refined_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> prior_pose_;

  std::string base_frame_;
  double staging_x_offset_{-0.63};
  double staging_y_offset_{0.0};
  double staging_yaw_offset_{0.0};
  double detection_timeout_{1.5};
  double translation_x_{-0.489};
  double translation_y_{-0.006};
  double axis_yaw_offset_{0.0};
  double position_filter_coef_{0.3};
  double yaw_filter_coef_{0.2};
  double max_position_jump_{0.10};
  double max_yaw_jump_{0.17};
  bool require_axis_tag_{true};
  double hold_distance_{0.30};
  double pre_dock_distance_{0.12};
  double pre_dock_max_lateral_{0.025};
  double pre_dock_max_yaw_{0.1745};
  double contact_voltage_{0.5};
};

}  // namespace bagheera_docking

#endif  // BAGHEERA_DOCKING__TAG_CHARGING_DOCK_HPP_
