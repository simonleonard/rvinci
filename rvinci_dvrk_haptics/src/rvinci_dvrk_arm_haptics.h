#ifndef RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_ARM_HAPTICS_H_
#define RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_ARM_HAPTICS_H_

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

namespace rvinci_dvrk_haptics {
class DvrkArmHaptics {
public:
  DvrkArmHaptics(std::string arm_name, ros::NodeHandle& nh,
                 const std::string& topic_prefix);

  void startBringup();

  bool isReady() const;
  void setOperatorPresent(bool operator_present);

private:
  const std::string arm_name_;

  ros::Subscriber current_state_sub_;
  ros::Subscriber position_sub_;

  ros::Publisher desired_state_pub_;
  ros::Publisher position_pub_;
  ros::Publisher wrench_pub_;

  bool arm_status_ready_ = false;
  bool operator_present_ = false;

  geometry_msgs::PoseStamped latest_arm_pose_;


  void onCurrentStateChange(const std_msgs::String& msg);
  void onCurrentPositionChange(const geometry_msgs::PoseStamped& msg);

  void updateArmMode();
  void setEffortMode();
  void setPositionMode();
};
} // namespace rvinci_dvrk_haptics

#endif // RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_ARM_HAPTICS_H_
