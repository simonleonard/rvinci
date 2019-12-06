#ifndef RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_
#define RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_

// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// Messages
#include <rvinci_input_msg/rvinci_input.h>

namespace rvinci_dvrk_haptics {

enum class InteractionMode {
  kPositionHold, kFreeMotion, kCartesianImpedance, kCameraRepositioning
};

class dvrk_wrench {
public:
  dvrk_wrench();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);

private:
  ros::NodeHandle nh_;

  ros::Subscriber rvinci_update_sub_;

  ros::Publisher wrench_pub_left_;
  ros::Publisher wrench_pub_right_;
};

} // namespace rvinci_dvrk_haptics

#endif // RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_
