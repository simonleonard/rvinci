#include "rvinci_dvrk_arm_haptics.h"

// Standard
#include <utility>

// Messages
#include <geometry_msgs/Wrench.h>

namespace rvinci_dvrk_haptics {
namespace {
constexpr int kQueueSize = 10;
}

DvrkArmHaptics::DvrkArmHaptics(std::string arm_name, ros::NodeHandle& nh,
                               const std::string& topic_prefix)
    : arm_name_(std::move(arm_name)) {
  current_state_sub_ =
      nh.subscribe(topic_prefix + "current_state", kQueueSize,
                   &DvrkArmHaptics::onCurrentStateChange, this);
  position_sub_ = nh.subscribe(topic_prefix + "position_cartesian_current", 1,
                               &DvrkArmHaptics::onCurrentPositionChange, this);

  desired_state_pub_ = nh.advertise<std_msgs::String>(
      topic_prefix + "set_desired_state", 1, true);
  position_pub_ = nh.advertise<geometry_msgs::Pose>(
      topic_prefix + "set_position_goal_cartesian", 1, false);
  wrench_pub_ = nh.advertise<geometry_msgs::Wrench>(
      topic_prefix + "set_wrench_body", kQueueSize, false);
}

void DvrkArmHaptics::startBringup() {
  std_msgs::String msg;
  msg.data = "READY";
  desired_state_pub_.publish(msg);
}

bool DvrkArmHaptics::isReady() const {
  return arm_status_ready_ && latest_arm_pose_.header.stamp > ros::Time(0);
}

void DvrkArmHaptics::onCurrentStateChange(const std_msgs::String& msg) {
  ROS_INFO("%s arm changed state to %s", arm_name_.c_str(), msg.data.c_str());

  arm_status_ready_ = msg.data == "READY";

  updateArmMode();
}

void DvrkArmHaptics::onCurrentPositionChange(
    const geometry_msgs::PoseStamped& msg) {
  ROS_ERROR_COND(msg.header.stamp == ros::Time(0),
                 "rvinci_dvrk_haptics received a pose with timestamp 0");

  bool is_first_pose = latest_arm_pose_.header.stamp == ros::Time(0);
  latest_arm_pose_ = msg;

  if (is_first_pose) updateArmMode();
}

void DvrkArmHaptics::setOperatorPresent(bool operator_present) {
  operator_present_ = operator_present;
  updateArmMode();
}

void DvrkArmHaptics::updateArmMode() {
  if (!isReady()) return;

  if (operator_present_) {
    setEffortMode();
  } else {
    setPositionMode();
  }
}

void DvrkArmHaptics::setEffortMode() {
  wrench_pub_.publish(geometry_msgs::Wrench());
}

void DvrkArmHaptics::setPositionMode() {
  ROS_ASSERT_MSG(latest_arm_pose_.header.stamp != ros::Time(0),
                 "Tried to set position mode with invalid pose");

  position_pub_.publish(latest_arm_pose_.pose);
}

} // namespace rvinci_dvrk_haptics