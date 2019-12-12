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

// Messages
#include <rvinci_input_msg/rvinci_input.h>
#include <rviz/properties/float_property.h>

// Local
#include "rvinci/rvinci_gui.h"

namespace Ogre {
class SceneNode;
class RenderWindow;
class Camera;
class Viewport;
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

  // Used to apply the proper dispairity to each eye
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
  /*!Camera position is either manually entered, or calculated by position
   * of the da Vinci grips when the camera pedal is activated. A vector is
   * calculated between the right and left grips. The translation of the
   * midpoint of this vector is added to the camera node position, and the
   * change in orientation of this vector is added to the orientation of the
   * camera node.
   */
  void cameraUpdate();
  //! Called after constructor
  void onInitialize() override;
  //! Override from rviz display class.
  void update(float, float) override;
protected Q_SLOTS:
  virtual void changeInputScale();
  //! Resets or initializes camera and 3D cursor positions.
  virtual void cameraReset();
  //! Sets up ROS subscribers and publishers
  virtual void pubSubSetup();
  //! Toggle for DVRK Gravity Compensation state
  virtual void gravityCompensation();

private:
  struct PerEyeData {
    bool prev_grab = false;

    Ogre::Camera* camera = nullptr;
    Ogre::Viewport* viewport = nullptr;

    Ogre::Vector3 input_pos{};
    Ogre::Vector3 input_change{};

    geometry_msgs::Pose cursor;
    geometry_msgs::Pose pose_before_clutch;

    ros::Publisher pub_robot_state;
    ros::Publisher pub_gravity_comp;
  };

  RvinciGui gui_;

  PerEyeData left_{};
  PerEyeData right_{};

  std::array<std::reference_wrapper<PerEyeData>, 2> eyes_ = {left_, right_};

  bool camera_mode_ = false;
  bool clutch_mode_ = false;

  Ogre::SceneNode* camera_node_ = nullptr;
  Ogre::Quaternion camera_quat_{};
  Ogre::RenderWindow* window_ = nullptr;

  Ogre::Vector3 initial_c_vect_{};
  Ogre::Vector3 camera_ipd_{0.03, 0.0, 0.0};
  Ogre::Vector3 camera_offset_{0.0, -3.0, 1.5};
  Ogre::Vector3 camera_pos_{};

  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
  ros::Subscriber input_sub_;
#pragma clang diagnostic pop
  ros::Publisher cursor_right_pub_;
  ros::Publisher cursor_left_pub_;

  ros::Publisher clutch_start_pub_;
  ros::Publisher clutch_end_pub_;

  std::unique_ptr<rviz::RosTopicProperty> prop_ros_topic_;
  std::unique_ptr<rviz::VectorProperty> prop_cam_focus_;
  std::unique_ptr<rviz::QuaternionProperty> property_camera_rot_;
  //  std::unique_ptr<rviz::BoolProperty> prop_manual_coords_;
  std::unique_ptr<rviz::VectorProperty> prop_camera_posit_;
  std::unique_ptr<rviz::VectorProperty> prop_input_scalar_;
  std::unique_ptr<rviz::BoolProperty> prop_gravity_comp_;
  std::unique_ptr<rviz::BoolProperty> prop_cam_reset_;
  std::unique_ptr<rviz::FloatProperty> prop_disparity_;

  std::unique_ptr<rviz::RenderWidget> render_widget_;

  //! Creates viewports and cameras.
  void cameraSetup();
  //! Called when input message received.
  /*!Contains primary input logic. Records input position and calculates change
   * in input position. Updates cursor position then sends data to camera
   * control and cursor publisher.
   */
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  //! Publishes cursor position and grip state to interaction cursor 3D display
  //! type.
  void publishCursorUpdate(int left_grab, int right_grab);
  //! Logic for grip state, used in interaction cursor 3D display type.
  static int getAGrip(bool grab, PerEyeData& eye);
};

} // namespace rvinci

#endif
