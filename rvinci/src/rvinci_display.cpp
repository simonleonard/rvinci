/* * Copyright (c) 2013, Willow Garage, Inc.
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
#include "rvinci/rvinci_display.h"

#include <string>

#include <QApplication>
#include <QDesktopWidget>

#include <boost/bind.hpp>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>

#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/package.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/window_manager_interface.h>

#include <rvinci_input_msg/rvinci_input.h>
#include <std_msgs/String.h>

namespace rvinci {
namespace {
constexpr size_t kLeft = 0;
constexpr size_t kRight = 1;
} // namespace

rvinciDisplay::rvinciDisplay() {

  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
      ROS_PACKAGE_NAME);

  prop_ros_topic_ = std::make_unique<rviz::RosTopicProperty>(
      "Input Topic", "/davinci_msg",
      ros::message_traits::datatype<rvinci_input_msg::rvinci_input>(),
      "Subscription topic (published by input controller node)", this,
      SLOT(pubSubSetup()));
  prop_input_scalar_ = std::make_unique<rviz::VectorProperty>(
      "Input Scalar", Ogre::Vector3(5, 5, 5),
      "Scalar for X, Y, and Z of controller input motion", this);
  prop_cam_reset_ = std::make_unique<rviz::BoolProperty>(
      "Camera Reset", false, "Reset camera and cursor position", this,
      SLOT(cameraReset()));
  prop_gravity_comp_ = std::make_unique<rviz::BoolProperty>(
      "Release da Vinci", false, "Put da Vinci in Gravity Compensation mode",
      this, SLOT(gravityCompensation()));
//  prop_manual_coords_ = std::make_unique<rviz::BoolProperty>(
//      "Use typed coordinates", false,
//      "Camera movement controlled by typed coordinates", this);
  prop_cam_focus_ = std::make_unique<rviz::VectorProperty>(
      "Camera Focus", Ogre::Vector3(0, 0, 0), "Focus Point of Camera", this);
  prop_camera_posit_ = std::make_unique<rviz::VectorProperty>(
      "Camera Position", camera_offset_,
      "Position of scene node to world base frame", this);
  property_camera_rot_ = std::make_unique<rviz::QuaternionProperty>(
      "Camera Orientation", Ogre::Quaternion(0, 0, 0, 1),
      "Orientation of the camera", this);
}

rvinciDisplay::~rvinciDisplay() {
  for (int i = 0; i < 2; ++i) {
    if (viewport_[i]) {
      window_->removeViewport(0);
      viewport_[i] = nullptr;
    }

    if (camera_[i]) {
      camera_[i]->getParentSceneNode()->detachObject(camera_[i]);
      scene_manager_->destroyCamera(camera_[i]);
      camera_[i] = nullptr;
    }
  }
  if (camera_node_) {
    camera_node_->getParentSceneNode()->removeChild(camera_node_);
    scene_manager_->destroySceneNode(camera_node_);
    camera_node_ = nullptr;
  }
  window_ = nullptr;
}

void rvinciDisplay::onInitialize() {
  render_widget_ =
      std::make_unique<rviz::RenderWidget>(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(2800, 1050);
  render_widget_->show();
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint |
                                 Qt::WindowTitleHint);
  window_ = render_widget_->getRenderWindow();
  window_->setVisible(false);
  window_->setAutoUpdated(false);
  window_->addListener(this);
  camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_manager_->getRootSceneNode()->createChildSceneNode();

  pubSubSetup();
}

void rvinciDisplay::update(float, float) {
  cameraUpdate();
  window_ = render_widget_->getRenderWindow();
  window_->update(false);
}

// void rvinciDisplay::reset(){}

void rvinciDisplay::pubSubSetup() {
  std::string subtopic = prop_ros_topic_->getStdString();
  input_sub_ = nh_.subscribe(subtopic, 10, &rvinciDisplay::inputCallback, this);
  cursor_right_pub_ =
      nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>(
          "rvinci_cursor_right/update", 10);
  cursor_left_pub_ =
      nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>(
          "rvinci_cursor_left/update", 10);
  pub_robot_state_[kLeft] =
      nh_.advertise<std_msgs::String>("dvrk/MTML/set_robot_state", 10);
  pub_robot_state_[kRight] =
      nh_.advertise<std_msgs::String>("dvrk/MTMR/set_robot_state", 10);
}

void rvinciDisplay::gravityCompensation() {
  std_msgs::String msg;
  if (prop_gravity_comp_->getBool()) {
    msg.data = "DVRK_GRAVITY_COMPENSATION";
  } else {
    msg.data = "DVRK_READY";
  }
  pub_robot_state_[kLeft].publish(msg);
  pub_robot_state_[kRight].publish(msg);
}

void rvinciDisplay::inputCallback(
    const rvinci_input_msg::rvinci_input::ConstPtr& r_input) {

  // shifts incoming davinci orientation into world frame
  Ogre::Quaternion ori_shift(0, 0, -sqrt(0.5), sqrt(0.5));
  ori_shift = ori_shift * Ogre::Quaternion(0, 0, 1, 0);
  Ogre::Quaternion in_ori[2];

  camera_mode_ = r_input->camera;
  clutch_mode_ = r_input->clutch;

  if (!clutch_mode_) {
    Ogre::Quaternion cam_ori = camera_[kLeft]->getRealOrientation();
    int grab[2];
    for (int i = 0; i < 2; ++i) {
      // getting absolute and delta position of grippers, for use in cam and
      // cursor.
      Ogre::Vector3 old_input = input_pos_[i];
      geometry_msgs::PoseStamped pose = r_input->gripper[i].pose;

      Ogre::Quaternion offset(0.7071, 0.7071, 0, 0);
      input_pos_[i] =
          offset * Ogre::Vector3(pose.pose.position.x, pose.pose.position.y,
                                 pose.pose.position.z);
      input_pos_[i] *= prop_input_scalar_->getVector();
      in_ori[i] =
          Ogre::Quaternion(pose.pose.orientation.w, pose.pose.orientation.x,
                           pose.pose.orientation.y, pose.pose.orientation.z);
      in_ori[i] = cam_ori * (ori_shift * in_ori[i]);

      input_change_[i] = cam_ori * (input_pos_[i] - old_input);
    }

    if (!camera_mode_) {
      geometry_msgs::Pose cursor_pose;

      for (int i = 0; i < 2; ++i) {
        cursor_[i].position.x += input_change_[i].x;
        cursor_[i].position.y += input_change_[i].y;
        cursor_[i].position.z += input_change_[i].z;
        cursor_[i].orientation.x = in_ori[i].x;
        cursor_[i].orientation.y = in_ori[i].y;
        cursor_[i].orientation.z = in_ori[i].z;
        cursor_[i].orientation.w = in_ori[i].w;
        grab[i] = getAGrip(r_input->gripper[i].grab, i);
      }
      publishCursorUpdate(grab);
      /*
       * inital_vect is constantly calculated, to set origin vector between
       * grippers when camera mode is triggered.
       */
      initial_c_vect_ = (input_pos_[kLeft] - input_pos_[kRight]);
      initial_c_vect_.normalise(); // normalise, otherwise issues when doing
                                   // v1.getRotationto(v2);
      camera_quat_ = camera_node_->getOrientation();
    } else {
      /*
       * When camera mode is activated, cursor positions are reinitialized
       * around the camera node, similar to the orientation of the grippers in
       * real life. This gives an intuitive feel, so when the cameras are moved,
       * the cursors appear on screen similarly to where your hands are
       * positioned in real life.
       */

      cameraUpdate();
      Ogre::Vector3 campos = camera_node_->getPosition();
      for (int& i : grab) {
        i = 0;
      }
      prop_cam_focus_->setVector(input_pos_[kRight]);
      publishCursorUpdate(grab);
    }
  } else {
    // to avoid an erroneously large input_update_ following clutched movement
    for (int i = 0; i < 2; ++i) {
      geometry_msgs::PoseStamped pose = r_input->gripper[i].pose;
      Ogre::Quaternion offset(0.7071, 0.7071, 0, 0);
      input_pos_[i] =
          offset * Ogre::Vector3(pose.pose.position.x, pose.pose.position.y,
                                 pose.pose.position.z);
      input_pos_[i] *= prop_input_scalar_->getVector();
    }
  }
}

void rvinciDisplay::publishCursorUpdate(const int grab[2]) {
  // fixed frame is a parent member from RViz Display, pointing to selected
  // world frame in rviz;
  std::string frame = context_->getFixedFrame().toStdString();
  interaction_cursor_msgs::InteractionCursorUpdate cursor_left;
  interaction_cursor_msgs::InteractionCursorUpdate cursor_right;

  cursor_right.pose.header.frame_id = frame;
  cursor_right.pose.header.stamp = ros::Time::now();
  cursor_right.pose.pose = cursor_[kRight];
  cursor_right.button_state = grab[kRight];

  cursor_left.pose.header.frame_id = frame;
  cursor_left.pose.header.stamp = ros::Time::now();
  cursor_left.pose.pose = cursor_[kLeft];
  cursor_left.button_state = grab[kLeft];

  cursor_right_pub_.publish(cursor_right);
  cursor_left_pub_.publish(cursor_left);
}

int rvinciDisplay::getAGrip(bool grab, int i) {
  if (grab && !prev_grab_[i]) {
    prev_grab_[i] = grab;
    return 2; // Grab object
  }
  if (grab && prev_grab_[i]) {
    prev_grab_[i] = grab;
    return 1; // hold object
  }
  if (!grab && prev_grab_[i]) {
    prev_grab_[i] = grab;
    return 3; // Release object
  }
  if (!grab && !prev_grab_[i]) {
    prev_grab_[i] = grab;
    return 0; // none
  }
}

void rvinciDisplay::cameraSetup() {
  Ogre::ColourValue bg_color = context_->getViewManager()
                                   ->getRenderPanel()
                                   ->getViewport()
                                   ->getBackgroundColour();
  window_ = render_widget_->getRenderWindow();
  camera_[kLeft] = scene_manager_->createCamera("Left Camera");
  camera_[kRight] = scene_manager_->createCamera("Right Camera");
  for (int i = 0; i < 2; ++i) {
    camera_node_->attachObject(camera_[i]);
    // static_cast silences a narrowing conversion warning
    viewport_[i] = window_->addViewport(
        camera_[i], i, 0.5f * static_cast<float>(i), 0.0f, 0.5f, 1.0f);
    viewport_[i]->setBackgroundColour(bg_color);
  }
  cameraReset();
}

void rvinciDisplay::cameraReset() {
  camera_pos_ = Ogre::Vector3(0.0f, 0.0f, 0.0f);
  camera_node_->setOrientation(1, 0, 0, 0);
  camera_node_->setPosition(camera_pos_);
  for (int i = 0; i < 2; ++i) {
    camera_[i]->setNearClipDistance(0.01f);
    camera_[i]->setFarClipDistance(10000.0f);
    camera_[i]->setFixedYawAxis(true, camera_node_->getOrientation() *
                                          Ogre::Vector3::UNIT_Z);
    camera_[i]->setPosition(camera_offset_ - camera_ipd_ + 2 * i * camera_ipd_);
    camera_[i]->lookAt(camera_node_->getPosition());

    cursor_[i].position.x = (2 * i - 1) * 0.6;
    cursor_[i].position.y = 0;
    cursor_[i].position.z = 0;
  }

  prop_cam_reset_->setValue(QVariant(false));
}

void rvinciDisplay::cameraUpdate() {
  if (camera_mode_) {
    Ogre::Vector3 new_vec = (input_pos_[kLeft] - input_pos_[kRight]);
    new_vec.normalise();

    Ogre::Quaternion offset(1, 0, 0, 0);
    Ogre::Quaternion cam_rot =
        (offset * initial_c_vect_).getRotationTo(offset * new_vec);
    camera_node_->setOrientation(camera_quat_ * cam_rot.Inverse());

    camera_pos_ = Ogre::Vector3(
        camera_pos_ - ((input_change_[kRight] + input_change_[kLeft])));
    camera_node_->setPosition(camera_pos_);

    property_camera_rot_->setQuaternion(camera_[kLeft]->getRealOrientation());
    prop_camera_posit_->setVector(camera_pos_ +
                                  property_camera_rot_->getQuaternion() *
                                      camera_[kLeft]->getPosition());
    prop_cam_focus_->setVector(camera_node_->getPosition());
  }
}

void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
  cameraUpdate();
}

void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
}

void rvinciDisplay::onEnable() {
  if (!camera_[kLeft]) {
    cameraSetup();
  }
  render_widget_->setVisible(true);
  cameraReset();
}

void rvinciDisplay::onDisable() { render_widget_->setVisible(false); }

} // namespace rvinci

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display) // NOLINT
