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
#include <OgrePanelOverlayElement.h>

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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace rvinci {

rvinciDisplay::rvinciDisplay() : gui_() {

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
      "Scalar for X, Y, and Z of controller input motion", this,
      SLOT(changeInputScale()));
  prop_cam_reset_ = std::make_unique<rviz::BoolProperty>(
      "Camera Reset", false, "Reset camera and cursor position", this,
      SLOT(cameraReset()));
  prop_gravity_comp_ = std::make_unique<rviz::BoolProperty>(
      "Gravity Compensation", false,
      "Put da Vinci in Gravity Compensation mode", this,
      SLOT(gravityCompensation()));
  prop_disparity_ = std::make_unique<rviz::FloatProperty>(
      "Disparity", 0.05, "Overlay GUI Disparity", this);
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
  for (PerEyeData& eye : eyes_) {
    if (eye.viewport) {
      window_->removeViewport(0);
      eye.viewport = nullptr;
    }

    if (eye.camera) {
      eye.camera->getParentSceneNode()->detachObject(eye.camera);
      scene_manager_->destroyCamera(eye.camera);
      eye.camera = nullptr;
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

  gui_.initialize(nh_, prop_input_scalar_->getVector());

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

  clutch_start_pub_ = nh_.advertise<std_msgs::Empty>("rvinci/clutch_start", 10);
  clutch_end_pub_ =
      nh_.advertise<geometry_msgs::Transform>("rvinci/clutch_end", 10);

  left_.pub_robot_state =
      nh_.advertise<std_msgs::String>("dvrk/MTML/set_robot_state", 10);
  right_.pub_robot_state =
      nh_.advertise<std_msgs::String>("dvrk/MTMR/set_robot_state", 10);

  left_.pub_gravity_comp =
      nh_.advertise<std_msgs::Bool>("dvrk/MTML/set_gravity_compensation", 10);
  right_.pub_gravity_comp =
      nh_.advertise<std_msgs::Bool>("dvrk/MTMR/set_gravity_compensation", 10);
}

void rvinciDisplay::gravityCompensation() {
  std_msgs::Bool msg;
  msg.data = prop_gravity_comp_->getBool();

  left_.pub_gravity_comp.publish(msg);
  right_.pub_gravity_comp.publish(msg);
}

void rvinciDisplay::inputCallback(
    const rvinci_input_msg::rvinci_input::ConstPtr& r_input) {

  // shifts incoming davinci orientation into world frame
  Ogre::Quaternion ori_shift(0, 0, -sqrt(0.5), sqrt(0.5));
  ori_shift = ori_shift * Ogre::Quaternion(0, 0, 1, 0);
  Ogre::Quaternion in_ori[2];

  bool was_clutch_mode = clutch_mode_;

  camera_mode_ = r_input->camera;
  clutch_mode_ = r_input->clutch;

  if (!clutch_mode_) {
    Ogre::Quaternion cam_ori = left_.camera->getRealOrientation();
    int grab[2];
    for (int i = 0; i < 2; ++i) {
      PerEyeData& eye = eyes_[i];
      // getting absolute and delta position of grippers, for use in cam and
      // cursor.
      Ogre::Vector3 old_input = eye.input_pos;
      geometry_msgs::PoseStamped pose = r_input->gripper[i].pose;

      Ogre::Quaternion offset(0.7071, 0.7071, 0, 0);
      eye.input_pos =
          offset * Ogre::Vector3(pose.pose.position.x, -pose.pose.position.y,
                                 -pose.pose.position.z);
      eye.input_pos *= prop_input_scalar_->getVector();
      in_ori[i] =
          Ogre::Quaternion(pose.pose.orientation.w, pose.pose.orientation.x,
                           pose.pose.orientation.y, pose.pose.orientation.z);
      in_ori[i] = cam_ori * (ori_shift * in_ori[i]);

      eye.input_change = cam_ori * (eye.input_pos - old_input);
    }

    if (!camera_mode_) {
      geometry_msgs::Pose cursor_pose;

      for (int i = 0; i < 2; ++i) {
        PerEyeData& eye = eyes_[i];
        eye.cursor.position.x += eye.input_change.x;
        eye.cursor.position.y += eye.input_change.y;
        eye.cursor.position.z += eye.input_change.z;
        eye.cursor.orientation.x = in_ori[i].x;
        eye.cursor.orientation.y = in_ori[i].y;
        eye.cursor.orientation.z = in_ori[i].z;
        eye.cursor.orientation.w = in_ori[i].w;
        grab[i] = getAGrip(r_input->gripper[i].grab, eye);
      }
      publishCursorUpdate(grab[0], grab[1]);
      /*
       * inital_vect is constantly calculated, to set origin vector between
       * grippers when camera mode is triggered.
       */
      initial_c_vect_ = (left_.input_pos - right_.input_pos);
      // normalise, otherwise issues when doing v1.getRotationto(v2);
      initial_c_vect_.normalise();
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
      prop_cam_focus_->setVector(right_.input_pos);
      publishCursorUpdate(0, 0);
    }
  } else {
    // to avoid an erroneously large input_update_ following clutched movement
    for (int i = 0; i < 2; ++i) {
      PerEyeData& eye = eyes_[i];
      geometry_msgs::PoseStamped pose = r_input->gripper[i].pose;
      Ogre::Quaternion offset(0.7071, 0.7071, 0, 0);
      eye.input_pos =
          offset * Ogre::Vector3(pose.pose.position.x, -pose.pose.position.y,
                                 -pose.pose.position.z);
      eye.input_pos *= prop_input_scalar_->getVector();
    }
  }

  Ogre::Vector3 camera_avg_position =
      (left_.camera->getRealPosition() + right_.camera->getRealPosition()) / 2.;
  Ogre::Quaternion camera_avg_orientation =
      Ogre::Quaternion::Slerp(0.5, left_.camera->getRealOrientation(),
                              right_.camera->getRealOrientation(), true);

  tf::StampedTransform camera_tf;
  camera_tf.setOrigin(tf::Point(camera_avg_position.x, camera_avg_position.y,
                                camera_avg_position.z));
  camera_tf.setBasis(tf::Matrix3x3(
      tf::Quaternion(camera_avg_orientation.x, camera_avg_orientation.y,
                     camera_avg_orientation.z, camera_avg_orientation.w)));
  camera_tf.stamp_ = ros::Time::now();
  camera_tf.frame_id_ = fixed_frame_.toStdString();
  camera_tf.child_frame_id_ = "rvinci_camera";

  tf_broadcaster_.sendTransform(camera_tf);

  if (!was_clutch_mode && clutch_mode_) {
    for (int i = 0; i < 2; ++i) {
      eyes_[i].get().pose_before_clutch = r_input->gripper[i].pose.pose;
    }

    clutch_start_pub_.publish(std_msgs::Empty());
  } else if (was_clutch_mode && !clutch_mode_) {
    // This case is when you exit clutch mode
    // We need to find the transform between this pose and pose before clutch
    // and publish it
    for (int i = 0; i < 2; ++i) {
      PerEyeData& eye = eyes_[i];
      tf::Pose pose_before, pose_after;
      tf::poseMsgToTF(eye.pose_before_clutch, pose_before);
      tf::poseMsgToTF(r_input->gripper[i].pose.pose, pose_after);

      geometry_msgs::Transform transform_msg;
      tf::transformTFToMsg(pose_after * pose_before.inverse(), transform_msg);

      clutch_end_pub_.publish(transform_msg);
    }
  }
}

void rvinciDisplay::publishCursorUpdate(int left_grab, int right_grab) {
  // fixed frame is a parent member from RViz Display, pointing to selected
  // world frame in rviz;
  std::string frame = context_->getFixedFrame().toStdString();
  interaction_cursor_msgs::InteractionCursorUpdate cursor_left;
  interaction_cursor_msgs::InteractionCursorUpdate cursor_right;

  cursor_left.pose.header.frame_id = frame;
  cursor_left.pose.header.stamp = ros::Time::now();
  cursor_left.pose.pose = left_.cursor;
  cursor_left.button_state = left_grab;

  cursor_right.pose.header.frame_id = frame;
  cursor_right.pose.header.stamp = ros::Time::now();
  cursor_right.pose.pose = right_.cursor;
  cursor_right.button_state = right_grab;

  cursor_left_pub_.publish(cursor_left);
  cursor_right_pub_.publish(cursor_right);
}

int rvinciDisplay::getAGrip(bool grab, PerEyeData& eye) {
  if (grab && !eye.prev_grab) {
    eye.prev_grab = grab;
    return 2; // Grab object
  }
  if (grab && eye.prev_grab) {
    eye.prev_grab = grab;
    return 1; // hold object
  }
  if (!grab && eye.prev_grab) {
    eye.prev_grab = grab;
    return 3; // Release object
  }
  if (!grab && !eye.prev_grab) {
    eye.prev_grab = grab;
    return 0; // none
  }
}

void rvinciDisplay::cameraSetup() {
  Ogre::ColourValue bg_color = context_->getViewManager()
                                   ->getRenderPanel()
                                   ->getViewport()
                                   ->getBackgroundColour();
  window_ = render_widget_->getRenderWindow();
  left_.camera = scene_manager_->createCamera("Left Camera");
  right_.camera = scene_manager_->createCamera("Right Camera");
  for (int i = 0; i < 2; ++i) {
    PerEyeData& eye = eyes_[i];

    eye.camera->addListener(this);
    camera_node_->attachObject(eye.camera);
    // static_cast silences a narrowing conversion warning
    eye.viewport = window_->addViewport(
        eye.camera, i, 0.5f * static_cast<float>(i), 0.0f, 0.5f, 1.0f);
    eye.viewport->setBackgroundColour(bg_color);
  }
  cameraReset();
}

void rvinciDisplay::cameraReset() {
  camera_pos_ = Ogre::Vector3(0.0f, 0.0f, 0.0f);
  camera_node_->setOrientation(1, 0, 0, 0);
  camera_node_->setPosition(camera_pos_);
  for (int i = 0; i < 2; ++i) {
    PerEyeData& eye = eyes_[i];
    eye.camera->setNearClipDistance(0.01f);
    eye.camera->setFarClipDistance(10000.0f);
    eye.camera->setFixedYawAxis(true, camera_node_->getOrientation() *
                                          Ogre::Vector3::UNIT_Z);
    eye.camera->setPosition(camera_offset_ - camera_ipd_ + 2 * i * camera_ipd_);
    eye.camera->lookAt(camera_node_->getPosition());

    eye.cursor.position.x = (2 * i - 1) * 0.6;
    eye.cursor.position.y = 0;
    eye.cursor.position.z = 0;
  }

  prop_cam_reset_->setValue(QVariant(false));
}

void rvinciDisplay::cameraUpdate() {
  if (camera_mode_) {
    Ogre::Vector3 new_vec = (left_.input_pos - right_.input_pos);
    new_vec.normalise();

    Ogre::Quaternion offset(1, 0, 0, 0);
    Ogre::Quaternion cam_rot =
        (offset * initial_c_vect_).getRotationTo(offset * new_vec);
    camera_node_->setOrientation(camera_quat_ * cam_rot.Inverse());

    camera_pos_ = Ogre::Vector3(camera_pos_ -
                                ((left_.input_change + right_.input_change)));
    camera_node_->setPosition(camera_pos_);

    property_camera_rot_->setQuaternion(left_.camera->getRealOrientation());
    prop_camera_posit_->setVector(camera_pos_ +
                                  property_camera_rot_->getQuaternion() *
                                      left_.camera->getPosition());
    prop_cam_focus_->setVector(camera_node_->getPosition());
  }
}

void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
  gui_.show();
  cameraUpdate();
}

void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
  gui_.hide();
}

void rvinciDisplay::onEnable() {
  if (!left_.camera) {
    cameraSetup();
  }

  render_widget_->setVisible(true);
  // The following two lines don't seem to work
  window_->setFullscreen(false, 1280 * 2, 1024);
  window_->reposition(1920 + 1920, 0);

  cameraReset();
}

void rvinciDisplay::onDisable() { render_widget_->setVisible(false); }

void rvinciDisplay::cameraPreRenderScene(Ogre::Camera* camera) {
  if (camera == left_.camera) {
    gui_.setOffset(-prop_disparity_->getFloat() / 2.);
  } else if (camera == right_.camera) {
    gui_.setOffset(prop_disparity_->getFloat() / 2.);
  }
}

void rvinciDisplay::changeInputScale() {
  gui_.changeInputScale(prop_input_scalar_->getVector());
}

} // namespace rvinci

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display) // NOLINT
