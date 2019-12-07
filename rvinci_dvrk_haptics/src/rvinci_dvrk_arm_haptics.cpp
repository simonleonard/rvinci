#include "rvinci_dvrk_arm_haptics.h"

// Standard
#include <utility>

// Messages
#include <cisst_msgs/prmCartesianImpedanceGains.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Wrench.h>

namespace rvinci_dvrk_haptics {
namespace {
constexpr int kQueueSize = 10;
}

DvrkArmHaptics::DvrkArmHaptics(std::string arm_name, ros::NodeHandle& nh,
                               tf::TransformListener& listener,
                               const std::string& topic_prefix)
    : arm_name_(std::move(arm_name)), tf_listener_(listener) {
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
  cartesian_impedance_pub_ =
      nh.advertise<cisst_msgs::prmCartesianImpedanceGains>(
          topic_prefix + "set_cartesian_impedance_gains", 1, true);
  debug_cartesian_impedance_pub_ = nh.advertise<geometry_msgs::PointStamped>(
      topic_prefix + "debug_cartesian_impedance_gains", 1, true);
}

void DvrkArmHaptics::startBringup() {
  std_msgs::String msg;
  msg.data = "READY";
  desired_state_pub_.publish(msg);
}

bool DvrkArmHaptics::isReady() const {
  return arm_status_ready_ && latest_arm_pose_.header.stamp > ros::Time(0);
}

tf::Point DvrkArmHaptics::getPositionWorld() const {
  if (latest_arm_pose_.header.stamp == ros::Time(0)) {
    throw std::runtime_error("Tried to get position before being ready");
  }

  tf::Stamped<tf::Pose> t;
  tf::poseStampedMsgToTF(latest_arm_pose_, t);
  tf_listener_.transformPose("world", t, t);

  return t.getOrigin();
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
  // DVRK is a bad citizen and publishes its transforms with an invalid frame
  latest_arm_pose_.header.frame_id += "_top_panel";

  if (is_first_pose) updateArmMode();
}

void DvrkArmHaptics::setOperatorPresent(bool operator_present) {
  operator_present_ = operator_present;
  updateArmMode();
}

void DvrkArmHaptics::updateArmMode() {
  if (!isReady()) return;

  if (camera_active_) return;

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

void DvrkArmHaptics::enterCameraMode() {
  camera_active_ = true;
  setEffortMode();
}

void DvrkArmHaptics::exitCameraMode() {
  camera_active_ = false;
  updateArmMode();
}

void DvrkArmHaptics::updateCameraHaptics(const tf::Point& position,
                                         const tf::Quaternion& orientation) {
  tf::Stamped<tf::Point> position_stamped(
      position, latest_arm_pose_.header.stamp, "world");
  tf_listener_.transformPoint(latest_arm_pose_.header.frame_id,
                              position_stamped, position_stamped);

  tf::Stamped<tf::Quaternion> orientation_stamped(
      orientation, latest_arm_pose_.header.stamp, "world");
  tf_listener_.transformQuaternion(latest_arm_pose_.header.frame_id,
                                   orientation_stamped, orientation_stamped);

  cisst_msgs::prmCartesianImpedanceGains msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = latest_arm_pose_.header.frame_id;
  // We don't need torque but it's best to provide valid messages
  msg.TorqueOrientation.w = 1;

  tf::vector3TFToMsg(position_stamped, msg.ForcePosition);
  tf::quaternionTFToMsg(orientation_stamped, msg.ForceOrientation);

  msg.PosStiffPos.x = -100;
  msg.PosStiffNeg.x = -100;
  
  msg.PosDampingNeg.x = -2;
  msg.PosDampingPos.x = -2;
  msg.PosDampingNeg.y = msg.PosDampingNeg.z = -0.5;
  msg.PosDampingPos.y = msg.PosDampingPos.z = -0.5;

  geometry_msgs::PointStamped dbg;
  dbg.header = latest_arm_pose_.header;
  tf::pointTFToMsg(position_stamped, dbg.point);
  debug_cartesian_impedance_pub_.publish(dbg);

  cartesian_impedance_pub_.publish(msg);
}

} // namespace rvinci_dvrk_haptics