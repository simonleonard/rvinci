#include "rvinci_dvrk_arm_haptics.h"

// Standard
#include <utility>

// Messages
#include <geometry_msgs/Wrench.h>

namespace rvinci_dvrk_haptics {
namespace {
constexpr int kQueueSize = 10;

cisst_msgs::prmCartesianImpedanceGains createZeroCartesianImpedance() {
  cisst_msgs::prmCartesianImpedanceGains msg;
  msg.header.stamp = ros::Time::now();

  msg.TorqueOrientation.w = 1.;
  msg.ForceOrientation.w = 1.;

  return msg;
}
} // namespace

DvrkArmHaptics::DvrkArmHaptics(std::string arm_name, ros::NodeHandle& nh,
                               tf::TransformListener& listener,
                               const std::string& topic_prefix)
    : arm_name_(std::move(arm_name)), tf_listener_(listener),
      input_cartesian_impedance_(createZeroCartesianImpedance()) {
  current_state_sub_ =
      nh.subscribe(topic_prefix + "current_state", kQueueSize,
                   &DvrkArmHaptics::onCurrentStateChange, this);
  arm_mode_sub_ = nh.subscribe(topic_prefix + "status", kQueueSize,
                               &DvrkArmHaptics::onArmModeChange, this);
  position_sub_ = nh.subscribe(topic_prefix + "position_cartesian_current", 1,
                               &DvrkArmHaptics::onCurrentPositionChange, this);
  input_cartesian_impedance_sub_ = nh.subscribe(
      topic_prefix + "rvinci_set_cartesian_impedance_gains", kQueueSize,
      &DvrkArmHaptics::onSetInputCartesianImpedance, this);
  pose_moving_camera_sub_ =
      nh.subscribe(topic_prefix + "rvinci_pose_moving_camera", kQueueSize,
                   &DvrkArmHaptics::onSetDesiredPoseMovingCamera, this);

  desired_state_pub_ = nh.advertise<std_msgs::String>(
      topic_prefix + "set_desired_state", 1, true);
  position_pub_ = nh.advertise<geometry_msgs::Pose>(
      topic_prefix + "set_position_goal_cartesian", 1, false);
  wrench_pub_ = nh.advertise<geometry_msgs::Wrench>(
      topic_prefix + "set_wrench_body", kQueueSize, false);
  cartesian_impedance_pub_ =
      nh.advertise<cisst_msgs::prmCartesianImpedanceGains>(
          topic_prefix + "set_cartesian_impedance_gains", 1, true);
  lock_orientation_pub_ = nh.advertise<geometry_msgs::Quaternion>(
      topic_prefix + "lock_orientation", 1, false);
  unlock_orientation_pub_ = nh.advertise<std_msgs::Empty>(
      topic_prefix + "unlock_orientation", 1, false);
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

void DvrkArmHaptics::setIsMovingCamera(bool is_moving_camera) {
  is_moving_camera_ = is_moving_camera;
}

void DvrkArmHaptics::setIsClutching(bool is_clutching) {
  is_clutching_ = is_clutching;
}

void DvrkArmHaptics::setOperatorPresent(bool operator_present) {
  operator_present_ = operator_present;
  update();
}

void DvrkArmHaptics::update() {
  ArmMode arm_mode = getArmMode();

  if (arm_mode == prev_arm_mode_) return;

  switch (arm_mode) {
  case ArmMode::kNotReady:
    break;

  case ArmMode::kCamera:
    enterCameraMode();
    break;

  case ArmMode::kClutching:
    enterClutchingMode();
    break;

  case ArmMode::kOperating:
    enterOperatingMode();
    break;

  case ArmMode::kIdle:
    enterIdleMode();
    break;
  }

  prev_arm_mode_ = arm_mode;
}

void DvrkArmHaptics::onCurrentStateChange(const std_msgs::String& msg) {
  ROS_INFO("%s arm changed state to %s", arm_name_.c_str(), msg.data.c_str());

  arm_status_ready_ = msg.data == "READY";

  update();
}

void DvrkArmHaptics::onArmModeChange(const std_msgs::String& msg) {
  // Sending a wrench (which we only ever do to enter effort mode) overwrites
  // our gains. This re-publishes them whenever we do that.
  if (getArmMode() == ArmMode::kOperating && msg.data.find("EFFORT_MODE")) {
    publishInputCartesianImpedance();
  }
}

void DvrkArmHaptics::onCurrentPositionChange(
    const geometry_msgs::PoseStamped& msg) {
  ROS_ERROR_COND(msg.header.stamp == ros::Time(0),
                 "rvinci_dvrk_haptics received a pose with timestamp 0");

  bool is_first_pose = latest_arm_pose_.header.stamp == ros::Time(0);
  latest_arm_pose_ = msg;
  // DVRK is a bad citizen and publishes its transforms with an invalid frame
  latest_arm_pose_.header.frame_id += "_top_panel";

  if (is_first_pose) update();
}

void DvrkArmHaptics::onSetInputCartesianImpedance(
    const cisst_msgs::prmCartesianImpedanceGains& msg) {
  // Producer should stop sending messages when we tell it we're clutching, but
  // there may still be messages in flight and we shouldn't store those.
  if (getArmMode() == ArmMode::kClutching) return;

  input_cartesian_impedance_ = msg;

  if (getArmMode() == ArmMode::kOperating) {
    cartesian_impedance_pub_.publish(input_cartesian_impedance_);
  }
}

void DvrkArmHaptics::onSetDesiredPoseMovingCamera(
    const geometry_msgs::PoseStamped& msg) {
  if (getArmMode() != ArmMode::kCamera) return;

  cisst_msgs::prmCartesianImpedanceGains gains;
  gains.header = msg.header;
  gains.ForcePosition.x = msg.pose.position.x;
  gains.ForcePosition.y = msg.pose.position.y;
  gains.ForcePosition.z = msg.pose.position.z;
  gains.ForceOrientation = msg.pose.orientation;

  // We don't use torque haptics but the quaternion has to be valid
  gains.TorqueOrientation.w = 1;

  gains.PosStiffPos.z = gains.PosStiffNeg.z = -200.;
  gains.PosDampingPos.z = gains.PosDampingNeg.z = -2.;

  cartesian_impedance_pub_.publish(gains);
  lock_orientation_pub_.publish(msg.pose.orientation);
}

DvrkArmHaptics::ArmMode DvrkArmHaptics::getArmMode() const {
  if (!isReady()) return ArmMode::kNotReady;

  if (is_moving_camera_) return ArmMode::kCamera;

  if (is_clutching_) return ArmMode::kClutching;

  if (operator_present_) return ArmMode::kOperating;

  return ArmMode::kIdle;
}
void DvrkArmHaptics::enterCameraMode() {
  // For camera mode, all that matters is that we're in effort mode -- the
  // camera update will overwrite whatever cartesian impedance there is
  if (latest_dvrk_mode_ != DvrkMode::kEffort) {
    activateEffortMode();
  }
  orientation_locked_ = true;
}

void DvrkArmHaptics::enterClutchingMode() {
  // Ensure we're in effort mode with no impedance. Setting effort mode clears
  // impedance, so we only have to disable impedance if we're not setting effort
  // mode. We also reset the input cartesian impedance -- it will be inaccurate
  // until the producer processes the clutch end event.
  input_cartesian_impedance_ = createZeroCartesianImpedance();
  if (latest_dvrk_mode_ != DvrkMode::kEffort) {
    activateEffortMode();
  } else {
    deactivateCartesianImpedance();
  }

  if (orientation_locked_) {
    unlockOrientation();
  }
}

void DvrkArmHaptics::enterOperatingMode() {
  // Ensure we're in effort mode with the input impedance set. Activating effort
  // mode clears impedance, so we need to set impedance after it, and to avoid
  // race conditions this happens when we receive the status change. In this
  // function we only need to publish the user's impedance if we were already
  // in effort mode.
  if (latest_dvrk_mode_ != DvrkMode::kEffort) {
    activateEffortMode();
  } else {
    publishInputCartesianImpedance();
  }

  if (orientation_locked_) {
    unlockOrientation();
  }
}

void DvrkArmHaptics::enterIdleMode() {
  // Just ensure position mode
  if (latest_dvrk_mode_ != DvrkMode::kPosition) {
    activatePositionMode();
  }
}

void DvrkArmHaptics::activateEffortMode() {
  wrench_pub_.publish(geometry_msgs::Wrench());
  latest_dvrk_mode_ = DvrkMode::kEffort;
}

void DvrkArmHaptics::activatePositionMode() {
  ROS_ASSERT_MSG(latest_arm_pose_.header.stamp != ros::Time(0),
                 "Tried to set position mode with invalid pose");

  position_pub_.publish(latest_arm_pose_.pose);
  latest_dvrk_mode_ = DvrkMode::kPosition;
}

void DvrkArmHaptics::deactivateCartesianImpedance() {
  cartesian_impedance_pub_.publish(createZeroCartesianImpedance());
}

void DvrkArmHaptics::publishInputCartesianImpedance() {
  input_cartesian_impedance_.header.stamp = ros::Time::now();
  cartesian_impedance_pub_.publish(input_cartesian_impedance_);
}

void DvrkArmHaptics::unlockOrientation() {
  unlock_orientation_pub_.publish(std_msgs::Empty());
  orientation_locked_ = true;
}

} // namespace rvinci_dvrk_haptics