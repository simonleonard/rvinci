#ifndef RVINCI_SRC_RVINCI_ARM_H_
#define RVINCI_SRC_RVINCI_ARM_H_

// ROS
#include <ros/ros.h>

// RViz/OGRE
#include <OgrePose.h>
#include <OgreVector3.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <rvinci_input_msg/Gripper.h>
#include <rviz/display_context.h>

namespace rvinci {

class RvinciArm {
public:
  void onInitialize(ros::NodeHandle& nh, const std::string& dvrk_topic_prefix,
                    const std::string& rvinci_topic_prefix);

  void setGravityCompensation(bool gravity_compensation);

  void startClutch(const geometry_msgs::Pose& arm_pose);
  void stopClutch(const geometry_msgs::Pose& arm_pose);

  void updateCursor(const std::string& fixed_frame,
                    const rvinci_input_msg::Gripper& gripper_msg,
                    const Ogre::Quaternion& camera_orientation,
                    const Ogre::Vector3& input_scale);

  void updateArmOnly(const rvinci_input_msg::Gripper& gripper_msg);

  // Returns where the cursor WOULD move to, but doesn't move it. (The camera
  // should move to put the cursors at these apparent positions.)
  std::pair<Ogre::Vector3, Ogre::Quaternion>
  updateCamera(const std::string& fixed_frame,
               const rvinci_input_msg::Gripper& gripper_msg,
               const Ogre::Quaternion& camera_orientation,
               const Ogre::Vector3& input_scale);

  void resetCursor(Ogre::Vector3 cursor_position);

private:
  bool prev_grab_ = false;
  Ogre::Vector3 prev_arm_position_;
  Ogre::Vector3 prev_cursor_position_;

  geometry_msgs::Pose arm_pose_before_clutch_;

  ros::Publisher robot_state_pub_;
  ros::Publisher gravity_comp_pub_;
  ros::Publisher cursor_update_pub_;
  ros::Publisher clutch_start_pub_;
  ros::Publisher clutch_end_pub_;

  Ogre::Vector3 getCursorPosition(const Ogre::Vector3& arm_position,
                                  const Ogre::Quaternion& camera_orientation,
                                  const Ogre::Vector3& input_scale);
  static Ogre::Quaternion
  getCursorOrientation(const Ogre::Quaternion& arm_orientation,
                       const Ogre::Quaternion& camera_orientation);

  uint8_t getGripperAction(bool grab);

  void publishCursorUpdate(const std::string& frame,
                           const Ogre::Vector3& cursor_position,
                           const Ogre::Quaternion& cursor_orientation,
                           uint8_t grip_action);
};
} // namespace rvinci
#endif // RVINCI_SRC_RVINCI_ARM_H_
