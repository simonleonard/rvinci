/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVINCI_DISPLAY_H
#define RVINCI_DISPLAY_H

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Qt
#include <QObject>

// RViz/Ogre
#include <OgreCamera.h>
#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>
#include <OgreRenderTargetListener.h>
#include <OgreVector3.h>
#include <rviz/display.h>
#include <rviz/properties/float_property.h>

// Messages
#include <cisst_msgs/prmCartesianImpedanceGains.h>
#include <rvinci_input_msg/rvinci_input.h>
#include <sensor_msgs/JointState.h>

// Local
#include "rvinci/rvinci_arm.h"
#include "rvinci/rvinci_gui.h"

namespace Ogre {
class SceneNode;
class RenderWindow;
class Camera;
class Overlay;
} // namespace Ogre

namespace rviz {
class BoolProperty;
class RenderWidget;
class VectorProperty;
class QuaternionProperty;
class RosTopicProperty;
} // namespace rviz

namespace rvinci {
class CameraDeleter {
public:
  void operator()(Ogre::Camera* camera);
};

class ViewportDeleter {
public:
  void operator()(Ogre::Viewport* viewport);
};

class rvinciDisplay : public rviz::Display,
                      public Ogre::RenderTargetListener,
                      public Ogre::Camera::Listener {
  //! RVinci display plugin for RViz.
  /*! The RVinci display class is a plugin for RViz which is designed to allow
   * a da Vinci surgical console to navigate the RViz environment and manipulate
   * virtual objects within the world. It spawns a separate window with stereo
   * display whose cameras can be controlled with the console. It also provides
   * outputs for the interaction_cursor_3D to spawn two 3D cursors.
   */
  Q_OBJECT

  using RvinciMessage = rvinci_input_msg::rvinci_input;

public:
  //! A constructor
  /*!The rviz/Qt Render Widget is created here, and the Ogre
   * rendering window is attached to it. The Ogre camera node
   * is spawned and the ROS subscriber and publisher setup member is called.
   */
  rvinciDisplay();
  //! Destructor
  ~rvinciDisplay() override;

  //  virtual void reset();

  //! Override from Ogre::RenderTargetListener
  void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) override;

  //! Override from Ogre::RenderTargetListener
  void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) override;

  // Used to apply the proper disparity to each eye
  void cameraPreRenderScene(Ogre::Camera* camera) override;

protected:
  //! Called after onInitialize.
  /*!Called after onInitialize or if display plugin is enabled
   * after being disabled. Calls camera setup member if cameras
   * are not initialized and makes external render window visible.
   */
  void onEnable() override;
  //! Called when plugin is disabled (by deselecting the check box).
  void onDisable() override;
  //! Contains primary logic for camera control.
  //! Called after constructor
  void onInitialize() override;
  //! Override from rviz display class.
  void update(float, float) override;

protected Q_SLOTS:
  virtual void onChangeInputScale();
  //! Resets or initializes camera and 3D cursor positions.
  virtual void onCameraReset();
  //! Sets up ROS subscribers and publishers
  virtual void onChangeInputTopic();
  //! Toggle for DVRK Gravity Compensation state
  virtual void onChangeGravityCompensation();

private:
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;

  RvinciGui gui_;
  RvinciArm left_arm_;
  RvinciArm right_arm_;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
  ros::Subscriber input_sub_;
  ros::Subscriber left_joint_state_sub_;
  ros::Subscriber right_joint_state_sub_;
#pragma clang diagnostic pop

  ros::Publisher left_desired_pose_pub_;
  ros::Publisher right_desired_pose_pub_;

  std::unique_ptr<rviz::RosTopicProperty> prop_ros_topic_;
  std::unique_ptr<rviz::VectorProperty> prop_cam_focus_;
  std::unique_ptr<rviz::QuaternionProperty> property_camera_rot_;
  std::unique_ptr<rviz::VectorProperty> prop_camera_posit_;
  std::unique_ptr<rviz::VectorProperty> prop_input_scalar_;
  std::unique_ptr<rviz::BoolProperty> prop_gravity_comp_;
  std::unique_ptr<rviz::BoolProperty> prop_cam_reset_;
  std::unique_ptr<rviz::FloatProperty> prop_disparity_;

  std::unique_ptr<rviz::RenderWidget> render_widget_;

  Ogre::SceneNode* camera_node_ = nullptr;

  std::unique_ptr<Ogre::Camera, CameraDeleter> left_camera_;
  std::unique_ptr<Ogre::Camera, CameraDeleter> right_camera_;
  std::unique_ptr<Ogre::Viewport, ViewportDeleter> left_viewport_;
  std::unique_ptr<Ogre::Viewport, ViewportDeleter> right_viewport_;

  double latest_left_roll_ = 0.;
  double latest_right_roll_ = 0.;
  tf::Pose rod_wrt_world_{};
  tf::Pose camera_wrt_dvrk_{};
  tf::Pose left_mtm_wrt_rod_{};
  tf::Pose right_mtm_wrt_rod_{};

  bool got_first_message_ = false;
  bool prev_clutch_ = false;
  bool prev_camera_ = false;

  //! Called when input message received.
  /*!Contains primary input logic. Records input position and calculates change
   * in input position. Updates cursor position then sends data to camera
   * control and cursor publisher.
   */
  void inputCallback(const RvinciMessage::ConstPtr& rvinci_msg);

  void updateClutching(const RvinciMessage& rvinci_msg);
  void broadcastCameraTransform();

  std::unique_ptr<Ogre::Camera, CameraDeleter>
  createCamera(const std::string& name);
  std::unique_ptr<Ogre::Viewport, ViewportDeleter>
  createViewport(Ogre::Camera* camera, int z_order, float left) const;

  void resetCamera(Ogre::Camera* camera, const Ogre::Vector3& position) const;
  tf::Pose getMtmPose(const rvinci_input_msg::Gripper& gripper_msg,
                      const tf::Vector3& input_scale) const;
  geometry_msgs::PoseStamped getCameraHaptics(const std::string& mtm_name, const ros::Time& stamp,
                   const tf::Vector3& input_scale,
                   const tf::Pose& haptics_pose);
};

} // namespace rvinci

#endif
