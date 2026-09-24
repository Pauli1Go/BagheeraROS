// Copyright 2026 Paul Praschl
// Licensed under the Apache License, Version 2.0.

#include "bagheera_docking/tag_charging_dock.hpp"

#include <cmath>
#include <cstdio>
#include <string>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bagheera_docking
{

namespace
{

// A docking run is a sequence of getRefinedPose calls without a long pause.
// The server calls getStagingPose once before navigation and once after the
// initial perception; only the first of those may reset the fused state.
constexpr double kRunIdleResetSeconds = 3.0;

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace

void TagChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }
  logger_ = node_->get_logger();
  dock_direction_ = opennav_docking_core::DockDirection::FORWARD;
  rotate_to_dock_ = false;

  auto declare = [this](const std::string & key, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key, value);
    };
  declare("staging_x_offset", rclcpp::ParameterValue(staging_x_offset_));
  declare("staging_y_offset", rclcpp::ParameterValue(staging_y_offset_));
  declare("staging_yaw_offset", rclcpp::ParameterValue(staging_yaw_offset_));
  declare("detection_timeout", rclcpp::ParameterValue(detection_timeout_));
  declare("external_detection_translation_x", rclcpp::ParameterValue(translation_x_));
  declare("external_detection_translation_y", rclcpp::ParameterValue(translation_y_));
  declare("axis_yaw_offset", rclcpp::ParameterValue(axis_yaw_offset_));
  declare("position_filter_coef", rclcpp::ParameterValue(position_filter_coef_));
  declare("yaw_filter_coef", rclcpp::ParameterValue(yaw_filter_coef_));
  declare("max_position_jump", rclcpp::ParameterValue(max_position_jump_));
  declare("max_yaw_jump", rclcpp::ParameterValue(max_yaw_jump_));
  declare("require_axis_tag", rclcpp::ParameterValue(require_axis_tag_));
  declare("hold_distance", rclcpp::ParameterValue(hold_distance_));
  declare("pre_dock_distance", rclcpp::ParameterValue(pre_dock_distance_));
  declare("pre_dock_max_lateral", rclcpp::ParameterValue(pre_dock_max_lateral_));
  declare("pre_dock_max_yaw", rclcpp::ParameterValue(pre_dock_max_yaw_));
  declare("event_topic", rclcpp::ParameterValue(std::string("/dock/plugin_event")));
  declare("contact_voltage", rclcpp::ParameterValue(contact_voltage_));
  declare("position_topic", rclcpp::ParameterValue(std::string("/dock/detected_pose")));
  declare("axis_topic", rclcpp::ParameterValue(std::string("/dock/detected_axis")));
  declare("power_topic", rclcpp::ParameterValue(std::string("/hardware_bridge/power")));
  declare("docked_topic", rclcpp::ParameterValue(std::string("/docked")));

  auto get = [this](const std::string & key, auto & value) {
      node_->get_parameter(name_ + "." + key, value);
    };
  get("staging_x_offset", staging_x_offset_);
  get("staging_y_offset", staging_y_offset_);
  get("staging_yaw_offset", staging_yaw_offset_);
  get("detection_timeout", detection_timeout_);
  get("external_detection_translation_x", translation_x_);
  get("external_detection_translation_y", translation_y_);
  get("axis_yaw_offset", axis_yaw_offset_);
  get("position_filter_coef", position_filter_coef_);
  get("yaw_filter_coef", yaw_filter_coef_);
  get("max_position_jump", max_position_jump_);
  get("max_yaw_jump", max_yaw_jump_);
  get("require_axis_tag", require_axis_tag_);
  get("hold_distance", hold_distance_);
  get("pre_dock_distance", pre_dock_distance_);
  get("pre_dock_max_lateral", pre_dock_max_lateral_);
  get("pre_dock_max_yaw", pre_dock_max_yaw_);
  get("contact_voltage", contact_voltage_);
  std::string position_topic, axis_topic, power_topic, docked_topic;
  get("position_topic", position_topic);
  get("axis_topic", axis_topic);
  get("power_topic", power_topic);
  get("docked_topic", docked_topic);
  std::string event_topic;
  get("event_topic", event_topic);
  node_->get_parameter("base_frame", base_frame_);

  position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    position_topic, 5,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_position_ = *msg;
    });
  axis_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    axis_topic, 5,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_axis_ = *msg;
    });
  power_sub_ = node_->create_subscription<mowgli_interfaces::msg::Power>(
    power_topic, rclcpp::SensorDataQoS(),
    [this](const mowgli_interfaces::msg::Power::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      charge_voltage_ = std::isfinite(msg->v_charge) ? msg->v_charge : 0.0;
    });
  docked_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    docked_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      docked_ = msg->data;
    });

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  staging_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);
  event_pub_ = node_->create_publisher<std_msgs::msg::String>(event_topic, 10);
}

void TagChargingDock::cleanup()
{
  position_sub_.reset();
  axis_sub_.reset();
  power_sub_.reset();
  docked_sub_.reset();
  dock_pose_pub_.reset();
  staging_pose_pub_.reset();
  event_pub_.reset();
}

void TagChargingDock::activate()
{
  dock_pose_pub_->on_activate();
  staging_pose_pub_->on_activate();
  event_pub_->on_activate();
}

void TagChargingDock::deactivate()
{
  dock_pose_pub_->on_deactivate();
  staging_pose_pub_->on_deactivate();
  event_pub_->on_deactivate();
}

bool TagChargingDock::runActive() const
{
  if (last_refine_call_.nanoseconds() == 0) {
    return false;
  }
  return (node_->now() - last_refine_call_).seconds() < kRunIdleResetSeconds;
}

void TagChargingDock::resetRun()
{
  tag_x_.reset();
  tag_y_.reset();
  axis_yaw_.reset();
  fused_frame_.clear();
  holding_ = false;
  pre_dock_rejected_ = false;
  refined_pose_ = geometry_msgs::msg::PoseStamped();
  last_position_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_axis_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_refine_call_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

geometry_msgs::msg::PoseStamped TagChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!runActive()) {
      resetRun();
      geometry_msgs::msg::PoseStamped prior;
      prior.header.frame_id = frame;
      prior.pose = pose;
      prior_pose_ = prior;
    }
  }

  // The staging pose is a fixed rigid offset from the dock: x along the dock
  // axis (negative = in front of it), y to the left, yaw relative to it.
  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = frame;
  staging.header.stamp = node_->now();
  staging.pose.position.x = pose.position.x +
    std::cos(yaw) * staging_x_offset_ - std::sin(yaw) * staging_y_offset_;
  staging.pose.position.y = pose.position.y +
    std::sin(yaw) * staging_x_offset_ + std::cos(yaw) * staging_y_offset_;
  staging.pose.orientation = yawToQuaternion(yaw + staging_yaw_offset_);
  staging_pose_pub_->publish(staging);
  return staging;
}

bool TagChargingDock::toFixedFrame(
  const geometry_msgs::msg::PoseStamped & in, const std::string & frame,
  geometry_msgs::msg::PoseStamped & out)
{
  try {
    // Use the transform at the image time: detections arrive ~0.9 s late and
    // the robot has moved since the frame was exposed.
    if (!tf2_buffer_->canTransform(
        frame, in.header.frame_id, in.header.stamp, rclcpp::Duration::from_seconds(0.1)))
    {
      return false;
    }
    tf2_buffer_->transform(in, out, frame);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000, "Dock detection transform failed: %s", ex.what());
    return false;
  }
  return true;
}

void TagChargingDock::updateAxis(const std::string & frame)
{
  const rclcpp::Time stamp(latest_axis_.header.stamp, RCL_ROS_TIME);
  if (latest_axis_.header.frame_id.empty() || stamp <= last_axis_stamp_ ||
    (node_->now() - stamp).seconds() > detection_timeout_)
  {
    return;
  }
  last_axis_stamp_ = stamp;
  geometry_msgs::msg::PoseStamped fixed;
  if (!toFixedFrame(latest_axis_, frame, fixed)) {
    return;
  }
  const double measured = angles::normalize_angle(
    tf2::getYaw(fixed.pose.orientation) + axis_yaw_offset_);
  if (!axis_yaw_) {
    axis_yaw_ = measured;
    RCLCPP_INFO(logger_, "Dock axis from ID 0: %.1f deg in %s",
      measured * 180.0 / M_PI, frame.c_str());
    return;
  }
  const double delta = angles::shortest_angular_distance(*axis_yaw_, measured);
  if (std::fabs(delta) > max_yaw_jump_) {
    RCLCPP_WARN(logger_, "Rejected ID 0 axis outlier: %.1f deg jump", delta * 180.0 / M_PI);
    return;
  }
  axis_yaw_ = angles::normalize_angle(*axis_yaw_ + yaw_filter_coef_ * delta);
}

bool TagChargingDock::updatePosition(const std::string & frame)
{
  const rclcpp::Time now = node_->now();
  const rclcpp::Time stamp(latest_position_.header.stamp, RCL_ROS_TIME);
  if (!latest_position_.header.frame_id.empty() && stamp > last_position_stamp_ &&
    (now - stamp).seconds() <= detection_timeout_)
  {
    geometry_msgs::msg::PoseStamped fixed;
    if (toFixedFrame(latest_position_, frame, fixed)) {
      last_position_stamp_ = stamp;
      const double x = fixed.pose.position.x;
      const double y = fixed.pose.position.y;
      if (!tag_x_) {
        tag_x_ = x;
        tag_y_ = y;
      } else if (std::hypot(x - *tag_x_, y - *tag_y_) > max_position_jump_) {
        RCLCPP_WARN(logger_, "Rejected ID 1 position outlier: %.3f m jump",
          std::hypot(x - *tag_x_, y - *tag_y_));
      } else {
        tag_x_ = *tag_x_ + position_filter_coef_ * (x - *tag_x_);
        tag_y_ = *tag_y_ + position_filter_coef_ * (y - *tag_y_);
      }
    }
  }
  return last_position_stamp_.nanoseconds() != 0 &&
         (now - last_position_stamp_).seconds() <= detection_timeout_;
}

bool TagChargingDock::robotPose(
  const std::string & frame, geometry_msgs::msg::PoseStamped & out)
{
  geometry_msgs::msg::PoseStamped base;
  base.header.frame_id = base_frame_;
  base.header.stamp = rclcpp::Time(0);
  base.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(base, out, frame);
  } catch (const tf2::TransformException &) {
    return false;
  }
  return true;
}

geometry_msgs::msg::PoseStamped TagChargingDock::composeDockPose() const
{
  double yaw = 0.0;
  if (axis_yaw_) {
    yaw = *axis_yaw_;
  } else if (prior_pose_) {
    yaw = tf2::getYaw(prior_pose_->pose.orientation);
  }
  geometry_msgs::msg::PoseStamped dock;
  dock.header.frame_id = fused_frame_;
  dock.header.stamp = node_->now();
  dock.pose.position.x = *tag_x_ + std::cos(yaw) * translation_x_ -
    std::sin(yaw) * translation_y_;
  dock.pose.position.y = *tag_y_ + std::sin(yaw) * translation_x_ +
    std::cos(yaw) * translation_y_;
  dock.pose.orientation = yawToQuaternion(yaw);
  return dock;
}

bool TagChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pre_dock_rejected_) {
    // Reported as a lost detection: the server resets to the staging pose
    // and tries again immediately instead of waiting for charge.
    pre_dock_rejected_ = false;
    return false;
  }
  const std::string frame = pose.header.frame_id;
  if (fused_frame_ != frame) {
    // Filters are only meaningful in a single fixed frame.
    tag_x_.reset();
    tag_y_.reset();
    axis_yaw_.reset();
    fused_frame_ = frame;
  }
  last_refine_call_ = node_->now();

  updateAxis(frame);
  const bool position_fresh = updatePosition(frame);

  if (!axis_yaw_) {
    if (require_axis_tag_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000, "Waiting for ID 0 to measure the dock axis");
      return false;
    }
    if (prior_pose_) {
      // Fallback yaw: dock database pose transformed into the fixed frame.
      geometry_msgs::msg::PoseStamped prior = *prior_pose_;
      prior.header.stamp = rclcpp::Time(0);
      try {
        tf2_buffer_->transform(prior, prior, frame);
        axis_yaw_ = tf2::getYaw(prior.pose.orientation);
        RCLCPP_WARN(logger_, "ID 0 not seen; using dock database yaw");
      } catch (const tf2::TransformException &) {
        return false;
      }
    }
  }
  if (!tag_x_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000, "Waiting for ID 1 to measure the dock position");
    return false;
  }

  if (!position_fresh) {
    // ID 1 leaves the image a few centimetres before contact. The dock is
    // static in the fixed frame, so the last estimate stays valid there.
    geometry_msgs::msg::PoseStamped robot;
    if (refined_pose_.header.frame_id != frame || !robotPose(frame, robot)) {
      return false;
    }
    const double distance = std::hypot(
      robot.pose.position.x - refined_pose_.pose.position.x,
      robot.pose.position.y - refined_pose_.pose.position.y);
    if (distance > hold_distance_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000, "ID 1 lost %.2f m before the dock", distance);
      return false;
    }
    if (!holding_) {
      RCLCPP_INFO(logger_, "ID 1 lost %.3f m before the dock; holding last pose", distance);
      holding_ = true;
    }
  } else {
    holding_ = false;
  }

  refined_pose_ = composeDockPose();
  dock_pose_pub_->publish(refined_pose_);
  // The server steers onto the pre-dock pose on the same axis, so its curve
  // is finished before the charging pins; the last centimetres are straight.
  pose = refined_pose_;
  const double yaw = tf2::getYaw(refined_pose_.pose.orientation);
  pose.pose.position.x -= std::cos(yaw) * pre_dock_distance_;
  pose.pose.position.y -= std::sin(yaw) * pre_dock_distance_;
  logDiagnostics(frame);
  return true;
}

void TagChargingDock::logDiagnostics(const std::string & frame)
{
  // One line per second: enough to reconstruct an approach from the log.
  const rclcpp::Time now = node_->now();
  if (last_diagnostic_.nanoseconds() != 0 && (now - last_diagnostic_).seconds() < 1.0) {
    return;
  }
  last_diagnostic_ = now;
  geometry_msgs::msg::PoseStamped robot;
  if (!robotPose(frame, robot)) {
    return;
  }
  const double yaw = tf2::getYaw(refined_pose_.pose.orientation);
  const double dx = robot.pose.position.x - refined_pose_.pose.position.x;
  const double dy = robot.pose.position.y - refined_pose_.pose.position.y;
  const double along = dx * std::cos(yaw) + dy * std::sin(yaw);
  const double left = -dx * std::sin(yaw) + dy * std::cos(yaw);
  const double yaw_error = angles::shortest_angular_distance(
    yaw, tf2::getYaw(robot.pose.orientation));
  const auto age = [&now](const rclcpp::Time & stamp) {
      return stamp.nanoseconds() == 0 ? -1.0 : (now - stamp).seconds();
    };
  RCLCPP_INFO(
    logger_,
    "Dock estimate: tag1=(%.3f, %.3f) axis=%.2f deg dock=(%.3f, %.3f) | robot to dock: "
    "along=%+.3f m left=%+.3f m yaw=%+.2f deg | age id1=%.2f s id0=%.2f s%s",
    *tag_x_, *tag_y_, yaw * 180.0 / M_PI, refined_pose_.pose.position.x,
    refined_pose_.pose.position.y, along, left, yaw_error * 180.0 / M_PI,
    age(last_position_stamp_), age(last_axis_stamp_), holding_ ? " HOLD" : "");
}

bool TagChargingDock::isDocked()
{
  geometry_msgs::msg::PoseStamped dock;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (docked_ || charge_voltage_ >= contact_voltage_) {
      return true;
    }
    if (!runActive() || refined_pose_.header.frame_id.empty()) {
      return false;
    }
    dock = refined_pose_;
  }

  // Hand-over to the straight final approach at the pre-dock line.
  geometry_msgs::msg::PoseStamped robot;
  if (!robotPose(dock.header.frame_id, robot)) {
    return false;
  }
  const double yaw = tf2::getYaw(dock.pose.orientation);
  const double along =
    (robot.pose.position.x - dock.pose.position.x) * std::cos(yaw) +
    (robot.pose.position.y - dock.pose.position.y) * std::sin(yaw);
  if (along >= -pre_dock_distance_) {
    const double left =
      -(robot.pose.position.x - dock.pose.position.x) * std::sin(yaw) +
      (robot.pose.position.y - dock.pose.position.y) * std::cos(yaw);
    const double yaw_error = angles::shortest_angular_distance(
      yaw, tf2::getYaw(robot.pose.orientation));
    char text[160];
    if (std::fabs(left) > pre_dock_max_lateral_ || std::fabs(yaw_error) > pre_dock_max_yaw_) {
      std::snprintf(
        text, sizeof(text), "PRE_DOCK_OFF_AXIS left %+.3f m (max %.3f), yaw %+.1f deg (max %.0f)",
        left, pre_dock_max_lateral_, yaw_error * 180.0 / M_PI, pre_dock_max_yaw_ * 180.0 / M_PI);
      RCLCPP_WARN(logger_, "Pre-dock pose off axis, retrying: %s", text);
      publishEvent(text);
      std::lock_guard<std::mutex> lock(mutex_);
      pre_dock_rejected_ = true;
      return false;
    }
    std::snprintf(
      text, sizeof(text), "PRE_DOCK_OK %.3f m before contact: left %+.3f m, yaw %+.1f deg",
      -along, left, yaw_error * 180.0 / M_PI);
    RCLCPP_INFO(logger_, "Pre-dock pose reached: %s", text);
    publishEvent(text);
    return true;
  }
  return false;
}

void TagChargingDock::publishEvent(const std::string & text)
{
  std_msgs::msg::String message;
  message.data = text;
  event_pub_->publish(message);
}

bool TagChargingDock::isCharging()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return docked_;
}

bool TagChargingDock::disableCharging()
{
  return true;
}

bool TagChargingDock::hasStoppedCharging()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !docked_ && charge_voltage_ < contact_voltage_;
}

}  // namespace bagheera_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bagheera_docking::TagChargingDock, opennav_docking_core::ChargingDock)
