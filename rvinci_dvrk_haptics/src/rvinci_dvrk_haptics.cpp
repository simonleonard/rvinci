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

  if (!is_camera_mode_ && rvinci_msg->camera) enterCameraMode();
  is_camera_mode_ = rvinci_msg->camera;

  left_arm_.setIsClutching(rvinci_msg->clutch);
  right_arm_.setIsClutching(rvinci_msg->clutch);

  left_arm_.setIsMovingCamera(rvinci_msg->camera);
  right_arm_.setIsMovingCamera(rvinci_msg->camera);

  left_arm_.update();
  right_arm_.update();
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
  desired_arm_distance_ =
      left_arm_.getPositionWorld().distance(right_arm_.getPositionWorld());
}

} // namespace rvinci_dvrk_haptics