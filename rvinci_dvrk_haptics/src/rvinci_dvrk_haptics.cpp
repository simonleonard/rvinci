#include "rvinci_dvrk_haptics.h"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

namespace rvinci_dvrk_haptics {
namespace {
using rvinci_input_msg::rvinci_input;

constexpr int kQueueSize = 10;
const tf::Vector3 kXAxis = tf::Vector3(1., 0., 0.);

} // namespace

DvrkHaptics::DvrkHaptics()
    : tf_listener_(nh_), left_arm_("Left", nh_, tf_listener_, "/dvrk/MTML/"),
      right_arm_("Right", nh_, tf_listener_, "/dvrk/MTMR/") {
  operator_present_sub_ =
      nh_.subscribe("/dvrk/footpedals/operatorpresent", kQueueSize,
                    &DvrkHaptics::onOperatorPresentChange, this);

  rvinci_update_sub_ = nh_.subscribe("/rvinci_input_update", kQueueSize,
                                     &DvrkHaptics::inputCallback, this);
}

void DvrkHaptics::inputCallback(const rvinci_input::ConstPtr& rvinci_msg) {
  // Ignore messages if not ready yet
  if (!left_arm_.isReady() || !right_arm_.isReady()) return;

  if (!is_camera_mode_ && rvinci_msg->camera) {
    enterCameraMode();
  } else if (is_camera_mode_ && !rvinci_msg->camera) {
    exitCameraMode();
  }

  if (is_camera_mode_) {
    //    tf::Point left_pos, right_pos;
    //    tf::pointMsgToTF(rvinci_msg->gripper[rvinci_input::LEFT].pose.pose.position,
    //                     left_pos);
    //    tf::pointMsgToTF(
    //        rvinci_msg->gripper[rvinci_input::RIGHT].pose.pose.position,
    //        right_pos);
    updateCameraHaptics(left_arm_.getPositionWorld(),
                        right_arm_.getPositionWorld());
  }
}

void DvrkHaptics::bringup() {
  left_arm_.startBringup();
  right_arm_.startBringup();
}

void DvrkHaptics::onOperatorPresentChange(const sensor_msgs::Joy& msg) {
  if (msg.buttons.empty()) {
    ROS_ERROR("Ignoring invalid operator present message");
    return;
  }

  bool operator_present = msg.buttons[0];
  left_arm_.setOperatorPresent(operator_present);
  right_arm_.setOperatorPresent(operator_present);
}

void DvrkHaptics::enterCameraMode() {
  is_camera_mode_ = true;

  desired_arm_distance_ =
      left_arm_.getPositionWorld().distance(right_arm_.getPositionWorld());
  left_arm_.enterCameraMode();
  right_arm_.enterCameraMode();
}

void DvrkHaptics::updateCameraHaptics(const tf::Point& left_pos,
                                      const tf::Point& right_pos) {
  updateCameraHapticsForArm(left_arm_, left_pos, right_arm_, right_pos);
  updateCameraHapticsForArm(right_arm_, right_pos, left_arm_, left_pos);
}

void DvrkHaptics::exitCameraMode() {
  is_camera_mode_ = false;

  left_arm_.exitCameraMode();
  right_arm_.exitCameraMode();
}

// Computes a set of cartesian impedance gains for |main_arm| that tries to
// keep it |desired_arm_distance_| distance from |other_arm|
void DvrkHaptics::updateCameraHapticsForArm(DvrkArmHaptics& main_arm,
                                            const tf::Point& main_arm_pos,
                                            DvrkArmHaptics& other_arm,
                                            const tf::Point& other_arm_pos) {
  ROS_INFO("Desired distance: %f", desired_arm_distance_);
  ROS_INFO("Actual distance: %f",
           main_arm.getPositionWorld().distance(other_arm.getPositionWorld()));

  // Find the point that main_arm would be at if it was at the desired distance
  tf::Vector3 other_to_main_unit = (main_arm_pos - other_arm_pos).normalized();
  tf::Point haptics_position =
      other_arm_pos + other_to_main_unit * desired_arm_distance_;

  // Find any orientation matrix that puts other_to_main_unit on one of the
  // axes (here, arbitrarily chosen to be x)
  tf::Vector3 axis = other_to_main_unit.cross(kXAxis);
  double angle = other_to_main_unit.angle(kXAxis);
  tf::Quaternion haptics_orientation(axis, angle);

  main_arm.updateCameraHaptics(haptics_position, haptics_orientation);
}

} // namespace rvinci_dvrk_haptics