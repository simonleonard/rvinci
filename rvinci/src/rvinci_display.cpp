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

// Standard/Boost
#include <boost/bind.hpp>
#include <string>

// ROS
#include <ros/package.h>
// The following two includes are needed, no matter what the IDE says, otherwise
// you get a linker error
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// RViz/OGRE
#include <OgrePanelOverlayElement.h>
#include <OgreRenderWindow.h>
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

// Qt
#include <QApplication>
#include <QDesktopWidget>

// Messages
#include <rvinci_input_msg/rvinci_input.h>
#include <std_msgs/Bool.h>

namespace rvinci {
namespace {
// Default pose of the camera node
const Ogre::Vector3 kDefaultCameraPosition = {-3., -1., 1.};
const Ogre::Quaternion kDefaultCameraOrientation = {Ogre::Degree(-90),
                                                    {0., 0., 1.}};

// 3d viewer angle. This accounts for the tilt of the viewer and is not part of
// the camera node's pose (it is applied to each camera).
const Ogre::Quaternion k3dViewerAngle = {Ogre::Radian(2.561), {1., 0., 0.}};

// Camera offset controls interpupillary distance
const Ogre::Vector3 kCameraOffset = {0.03, 0., 0.};

// Cursor offset determines where cursors are positioned by default. This is
// always reflected about the origin. The value is negative because the default
// pose is looking from the -x axis, if it was +x the value would be positive.
const Ogre::Vector3 kCursorOffset = {0., -0.5, 0.};

tf::Point pointOgreToTf(const Ogre::Vector3& p) { return {p.x, p.y, p.z}; }

tf::Quaternion quaternionOgreToTf(const Ogre::Quaternion& q) {
  return {q.x, q.y, q.z, q.w};
}

tf::Pose poseOgreToTf(const Ogre::Vector3& position,
                      const Ogre::Quaternion& orientation) {
  return tf::Pose(quaternionOgreToTf(orientation), pointOgreToTf(position));
}

Ogre::Vector3 pointTfToOgre(const tf::Point& p) {
  return {static_cast<Ogre::Real>(p.x()), static_cast<Ogre::Real>(p.y()),
          static_cast<Ogre::Real>(p.z())};
}

Ogre::Quaternion quaternionTfToOgre(const tf::Quaternion& q) {
  return {static_cast<Ogre::Real>(q.w()), static_cast<Ogre::Real>(q.x()),
          static_cast<Ogre::Real>(q.y()), static_cast<Ogre::Real>(q.z())};
}

tf2::Vector3 tfToTf2(const tf::Vector3& v) { return {v.x(), v.y(), v.z()}; }

tf2::Matrix3x3 tfToTf2(const tf::Matrix3x3& m) {
  return {m[0][0], m[0][1], m[0][2], m[1][0], m[1][1],
          m[1][2], m[2][0], m[2][1], m[2][2]};
}

tf2::Transform tfToTf2(const tf::Pose& pose) {
  return tf2::Transform{tfToTf2(pose.getBasis()), tfToTf2(pose.getOrigin())};
}

// Has to have this type (including boost and the ConstPtr-ness) to match one
// of the signatures of nh.subscribe()
boost::function<void(const sensor_msgs::JointState::ConstPtr&)>
jointStateCallback(double* target) {
  return [target](const sensor_msgs::JointState::ConstPtr& msg) {
    auto roll_it = std::find(msg->name.begin(), msg->name.end(), "wrist_roll");
    if (roll_it == msg->name.end()) {
      ROS_WARN_THROTTLE(2, "Couldn't find wrist_roll in joint state message");
      return;
    }

    const size_t roll_idx = std::distance(msg->name.begin(), roll_it);
    if (roll_idx >= msg->position.size()) {
      ROS_WARN_THROTTLE(2, "Malformed JointState message. The position vector "
                           "must be the same length as the name vector.");
      return;
    }

    *target = msg->position[roll_idx];
  };
}

double getRotationAngleAboutAxis(const tf::Quaternion& rotation,
                                 const tf::Vector3& axis) {
  return rotation.getAngle() * rotation.getAxis().dot(axis);
}

} // namespace

void CameraDeleter::operator()(Ogre::Camera* camera) {
  camera->getSceneManager()->destroyCamera(camera);
}

void ViewportDeleter::operator()(Ogre::Viewport* viewport) {
  viewport->getTarget()->removeViewport(viewport->getZOrder());
}

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
      SLOT(onChangeInputTopic()));
  prop_input_scalar_ = std::make_unique<rviz::VectorProperty>(
      "Input Scalar", Ogre::Vector3(5, 5, 5),
      "Scalar for X, Y, and Z of controller input motion", this,
      SLOT(onChangeInputScale()));
  prop_cam_reset_ = std::make_unique<rviz::BoolProperty>(
      "Camera Reset", false, "Reset camera and cursor position", this,
      SLOT(onCameraReset()));
  prop_gravity_comp_ = std::make_unique<rviz::BoolProperty>(
      "Gravity Compensation", false,
      "Put da Vinci in Gravity Compensation mode", this,
      SLOT(onChangeGravityCompensation()));
  prop_disparity_ = std::make_unique<rviz::FloatProperty>(
      "Disparity", 0.05, "Overlay GUI Disparity", this);
  prop_cam_focus_ = std::make_unique<rviz::VectorProperty>(
      "Camera Focus", Ogre::Vector3(0, 0, 0), "Focus Point of Camera", this);
  prop_camera_posit_ = std::make_unique<rviz::VectorProperty>(
      "Camera Position", kDefaultCameraPosition,
      "Position of scene node to world base frame", this);
  property_camera_rot_ = std::make_unique<rviz::QuaternionProperty>(
      "Camera Orientation", Ogre::Quaternion(0, 0, 0, 1),
      "Orientation of the camera", this);
}

rvinciDisplay::~rvinciDisplay() {
  if (scene_manager_ && camera_node_) {
    camera_node_->getParentSceneNode()->removeChild(camera_node_);
    scene_manager_->destroySceneNode(camera_node_);
    camera_node_ = nullptr;
  }
}

void rvinciDisplay::onInitialize() {
  gui_.initialize(nh_, prop_input_scalar_->getVector());

  left_arm_.onInitialize(nh_, "dvrk/MTML/", "rvinci/cursor_left/");
  right_arm_.onInitialize(nh_, "dvrk/MTMR/", "rvinci/cursor_right/");

  render_widget_ =
      std::make_unique<rviz::RenderWidget>(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(2800, 1050);
  render_widget_->show();
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint |
                                 Qt::WindowTitleHint);

  Ogre::RenderWindow* window = render_widget_->getRenderWindow();
  window->setVisible(false);
  window->setAutoUpdated(false);
  window->addListener(this);

  camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  onChangeInputTopic();
}

void rvinciDisplay::update(float, float) {
  render_widget_->getRenderWindow()->update(false);
}

// void rvinciDisplay::reset(){}

void rvinciDisplay::onChangeInputTopic() {
  input_sub_ = nh_.subscribe(prop_ros_topic_->getTopicStd(), 10,
                             &rvinciDisplay::inputCallback, this);

  left_joint_state_sub_ = nh_.subscribe("dvrk/MTML/state_joint_current", 10,
                                        jointStateCallback(&latest_left_roll_));
  right_joint_state_sub_ =
      nh_.subscribe("dvrk/MTMR/state_joint_current", 10,
                    jointStateCallback(&latest_right_roll_));

  left_desired_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "dvrk/MTML/rvinci_pose_moving_camera", 10, false);
  right_desired_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "dvrk/MTMR/rvinci_pose_moving_camera", 10, false);
}

void rvinciDisplay::onChangeGravityCompensation() {
  left_arm_.setGravityCompensation(prop_gravity_comp_->getBool());
  right_arm_.setGravityCompensation(prop_gravity_comp_->getBool());
}

void rvinciDisplay::inputCallback(const RvinciMessage::ConstPtr& rvinci_msg) {
  const rvinci_input_msg::Gripper& left_msg =
      rvinci_msg->gripper[RvinciMessage::LEFT];
  const rvinci_input_msg::Gripper& right_msg =
      rvinci_msg->gripper[RvinciMessage::RIGHT];

  if (!got_first_message_) {
    // Arms need to be initialized with first arm position
    left_arm_.updateArmOnly(left_msg);
    right_arm_.updateArmOnly(right_msg);

    // Note prev_clutch_ and prev_camera_ should NOT be set here. If clutch and/
    // or camera are pressed at the start, we act like they were just pressed.

    got_first_message_ = true;
    return;
  }

  const Ogre::Quaternion& camera_orientation = camera_node_->getOrientation();
  const Ogre::Vector3& input_scale = prop_input_scalar_->getVector();

  updateClutching(*rvinci_msg);

  if (rvinci_msg->clutch) {
    left_arm_.updateArmOnly(left_msg);
    right_arm_.updateArmOnly(right_msg);
  } else if (rvinci_msg->camera) {
    // To move camera we imagine a cylindrical rod with endpoints fixed at the
    // hands. The relative hand position controls the pitch and yaw of the rod,
    // and the hand orientation controls the roll.

    std::pair<Ogre::Vector3, Ogre::Quaternion> left_cursor =
        left_arm_.updateCamera(fixed_frame_.toStdString(), left_msg,
                               camera_orientation, input_scale);

    std::pair<Ogre::Vector3, Ogre::Quaternion> right_cursor =
        right_arm_.updateCamera(fixed_frame_.toStdString(), right_msg,
                                camera_orientation, input_scale);

    tf::Vector3 input_scale_tf(input_scale.x, input_scale.y, input_scale.z);
    tf::Pose left_mtm_pose = getMtmPose(left_msg, input_scale_tf);
    tf::Pose right_mtm_pose = getMtmPose(right_msg, input_scale_tf);

    tf::Point mtm_center_point =
        (left_mtm_pose.getOrigin() + right_mtm_pose.getOrigin()) / 2.;
    tf::Vector3 rod_axis =
        right_mtm_pose.getOrigin() - left_mtm_pose.getOrigin();
    rod_axis.normalize();
    tf::Quaternion mtm_orientation_from_position = quaternionOgreToTf(
        Ogre::Vector3::UNIT_X.getRotationTo(pointTfToOgre(rod_axis)));

    // Find the average of the rolls with the right roll negated, since the rod
    // axis is defined relative to the left hand.
    // TODO Try this with the roll values weighted by the scalar projection of
    //  the roll axis onto the rod axis. That should make roll not do anything
    //  in cases where it doesn't make sense, and should handle negating the
    //  right roll inherently.
    tf::Quaternion mtm_orientation_from_roll(
        rod_axis, (latest_left_roll_ - latest_right_roll_) / 2.);

    tf::Quaternion mtm_orientation =
        mtm_orientation_from_roll * mtm_orientation_from_position;
    tf::Pose rod_wrt_dvrk(mtm_orientation, mtm_center_point);

    if (!prev_camera_) {
      tf::Point cursor_center_point =
          pointOgreToTf((left_cursor.first + right_cursor.first) / 2.);

      // MTM center is defined to have its midpoint at the midpoint of the mtm
      // positions and its orientation equal to the DVRK frame.
      tf::Pose mtm_center_wrt_dvrk(tf::Matrix3x3::getIdentity(),
                                   mtm_center_point);
      // Cursor center is defined to have its midpoint at the midpoint of the
      // cursor positions and its orientation equal to the camera orientation
      tf::Pose cursor_center_wrt_world(quaternionOgreToTf(camera_orientation),
                                       cursor_center_point);
      // We create a mapping from DVRK frame to world frame that places the
      // MTM center at the cursor center at the moment you start moving the
      // camera
      tf::Pose dvrk_wrt_world =
          cursor_center_wrt_world * mtm_center_wrt_dvrk.inverse();
      rod_wrt_world_ = dvrk_wrt_world * rod_wrt_dvrk;

      tf::Pose camera_wrt_world(quaternionOgreToTf(camera_orientation),
                                pointOgreToTf(camera_node_->getPosition()));
      camera_wrt_dvrk_ = dvrk_wrt_world.inverse() * camera_wrt_world;

      left_mtm_wrt_rod_ = rod_wrt_dvrk.inverse() * left_mtm_pose;
      right_mtm_wrt_rod_ = rod_wrt_dvrk.inverse() * right_mtm_pose;
    } else {
      tf::Pose camera_wrt_world =
          rod_wrt_world_ * rod_wrt_dvrk.inverse() * camera_wrt_dvrk_;

      camera_node_->setPosition(pointTfToOgre(camera_wrt_world.getOrigin()));
      camera_node_->setOrientation(
          quaternionTfToOgre(camera_wrt_world.getRotation()));

      left_desired_pose_pub_.publish(
          getCameraHaptics("MTML", left_msg.pose.header.stamp, input_scale_tf,
                           rod_wrt_dvrk * left_mtm_wrt_rod_));
      right_desired_pose_pub_.publish(
          getCameraHaptics("MTMR", right_msg.pose.header.stamp, input_scale_tf,
                           rod_wrt_dvrk * right_mtm_wrt_rod_));
    }

    tf_broadcaster_.sendTransform({rod_wrt_world_, ros::Time::now(),
                                   fixed_frame_.toStdString(), "dvrk_mapping"});

  } else {
    left_arm_.updateCursor(fixed_frame_.toStdString(), left_msg,
                           camera_orientation, input_scale);
    right_arm_.updateCursor(fixed_frame_.toStdString(), right_msg,
                            camera_orientation, input_scale);
  }

  prev_camera_ = rvinci_msg->camera;

  broadcastCameraTransform();
}

tf::Pose rvinciDisplay::getMtmPose(const rvinci_input_msg::Gripper& gripper_msg,
                                   const tf::Vector3& input_scale) const {
  geometry_msgs::PoseStamped pose = gripper_msg.pose;
  pose.header.frame_id += "_top_panel";
  pose = context_->getTF2BufferPtr()->transform(pose, "world");
  tf::Pose pose_out;
  tf::poseMsgToTF(pose.pose, pose_out);
  pose_out.getOrigin() *= input_scale;
  return pose_out;
}

void rvinciDisplay::updateClutching(const RvinciMessage& rvinci_msg) {
  if (rvinci_msg.clutch && !prev_clutch_) {
    // Entering clutch
    left_arm_.startClutch(rvinci_msg.gripper[RvinciMessage::LEFT].pose.pose);
    right_arm_.startClutch(rvinci_msg.gripper[RvinciMessage::RIGHT].pose.pose);

    prev_clutch_ = true;
  } else if (!rvinci_msg.clutch && prev_clutch_) {
    // Exiting clutch
    left_arm_.stopClutch(rvinci_msg.gripper[RvinciMessage::LEFT].pose.pose);
    right_arm_.stopClutch(rvinci_msg.gripper[RvinciMessage::RIGHT].pose.pose);

    prev_clutch_ = false;
  }
}

void rvinciDisplay::broadcastCameraTransform() {
  tf::Pose camera_pose(quaternionOgreToTf(camera_node_->getOrientation()),
                       pointOgreToTf(camera_node_->getPosition()));

  tf_broadcaster_.sendTransform({camera_pose, ros::Time::now(),
                                 fixed_frame_.toStdString(), "rvinci_camera"});
}

void rvinciDisplay::onCameraReset() {
  camera_node_->setOrientation(kDefaultCameraOrientation);
  camera_node_->setPosition(kDefaultCameraPosition);

  // This can be called even if nullptr is passed
  resetCamera(left_camera_.get(), -kCameraOffset);
  resetCamera(right_camera_.get(), kCameraOffset);

  left_arm_.resetCursor(-kCursorOffset);
  right_arm_.resetCursor(kCursorOffset);

  prop_cam_reset_->setValue(QVariant(false));
}

void rvinciDisplay::resetCamera(Ogre::Camera* camera,
                                const Ogre::Vector3& position) const {
  if (!camera) return;

  camera->setNearClipDistance(0.01f);
  camera->setFarClipDistance(10000.0f);
  camera->setFixedYawAxis(true, camera_node_->getOrientation() *
                                    Ogre::Vector3::UNIT_Z);
  camera->setPosition(position);
  // Direction accepts a vector relative to the world and then transforms it to
  // be relative to the parent, so the direction we give it must be relative to
  // the parent.
  camera->setDirection(k3dViewerAngle * camera_node_->getOrientation() *
                       Ogre::Vector3::UNIT_Y);
}

void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
  gui_.show();
}

void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) {
  render_widget_->getRenderWindow()->swapBuffers();
  gui_.hide();
}

void rvinciDisplay::cameraPreRenderScene(Ogre::Camera* camera) {
  if (camera == left_camera_.get()) {
    gui_.setOffset(-prop_disparity_->getFloat() / 2.);
  } else if (camera == right_camera_.get()) {
    gui_.setOffset(prop_disparity_->getFloat() / 2.);
  }
}

void rvinciDisplay::onEnable() {
  left_camera_ = createCamera("Left Camera");
  left_viewport_ = createViewport(left_camera_.get(), 0, 0.0f);

  right_camera_ = createCamera("Right Camera");
  right_viewport_ = createViewport(right_camera_.get(), 1, 0.5f);

  render_widget_->setVisible(true);
  // The following two lines don't seem to work
  render_widget_->getRenderWindow()->setFullscreen(false, 1280 * 2, 1024);
  render_widget_->getRenderWindow()->reposition(1920 + 1920, 0);

  onCameraReset();
}

void rvinciDisplay::onDisable() {
  render_widget_->setVisible(false);

  left_viewport_ = nullptr;
  left_camera_ = nullptr;
  right_viewport_ = nullptr;
  right_camera_ = nullptr;
}

void rvinciDisplay::onChangeInputScale() {
  gui_.changeInputScale(prop_input_scalar_->getVector());
}

std::unique_ptr<Ogre::Camera, CameraDeleter>
rvinciDisplay::createCamera(const std::string& name) {
  Ogre::Camera* camera = scene_manager_->createCamera(name);
  camera->addListener(this);
  camera_node_->attachObject(camera);

  return std::unique_ptr<Ogre::Camera, CameraDeleter>(camera);
}

std::unique_ptr<Ogre::Viewport, ViewportDeleter>
rvinciDisplay::createViewport(Ogre::Camera* camera, int z_order,
                              float left) const {
  Ogre::Viewport* viewport = render_widget_->getRenderWindow()->addViewport(
      camera, z_order, left, 0.f, 0.5f, 1.0f);
  viewport->setBackgroundColour(context_->getViewManager()
                                    ->getRenderPanel()
                                    ->getViewport()
                                    ->getBackgroundColour());

  return std::unique_ptr<Ogre::Viewport, ViewportDeleter>(viewport);
}

geometry_msgs::PoseStamped rvinciDisplay::getCameraHaptics(
    const std::string& mtm_name, const ros::Time& stamp,
    const tf::Vector3& input_scale, const tf::Pose& haptics_pose) {
  // I don't actually need the geometry_msgs::Pose, but TF2 is apparently
  // incapable of transforming values of its OWN data types.
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = stamp;
  tf::poseTFToMsg(haptics_pose, pose.pose);

  // These poses are in scaled coordinates. Need to de-scale them.
  pose.pose.position.x /= input_scale.x();
  pose.pose.position.y /= input_scale.y();
  pose.pose.position.z /= input_scale.z();

  return context_->getTF2BufferPtr()->transform(pose, mtm_name + "_top_panel");
}

} // namespace rvinci

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display) // NOLINT
