/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rotors_gazebo_plugins/gazebo_view_control_plugin.h>

namespace gazebo {

GazeboViewControlPlugin::GazeboViewControlPlugin():
    GUIPlugin(),
    is_tracking_(false),
    display_forces_(false) {
  // Set the frame background and foreground colors
  setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 0); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *forward_button = new QPushButton(tr("Forward"));
  QPushButton *chase_button = new QPushButton(tr("Chase"));
  QPushButton *orthogonal_button = new QPushButton(tr("Orthogonal"));
  QPushButton *camera_button = new QPushButton(tr("Camera"));
  QPushButton *stop_button = new QPushButton(tr("Stop Tracking"));
  QPushButton *display_forces_button = new QPushButton(tr("Show Forces"));
  connect(forward_button, SIGNAL(clicked()), this, SLOT(OnForwardButton()));
  connect(chase_button, SIGNAL(clicked()), this, SLOT(OnChaseButton()));
  connect(orthogonal_button, SIGNAL(clicked()), this, SLOT(OnOrthogonalButton()));
  connect(camera_button, SIGNAL(clicked()), this, SLOT(OnCameraButton()));
  connect(stop_button, SIGNAL(clicked()), this, SLOT(OnStopButton()));
  connect(display_forces_button, SIGNAL(clicked()), this, SLOT(OnShowForces()));

  // Add the buttons to the frame's layout
  frameLayout->addWidget(forward_button);
  frameLayout->addWidget(chase_button);
  frameLayout->addWidget(orthogonal_button);
  frameLayout->addWidget(camera_button);
  frameLayout->addWidget(stop_button);
  frameLayout->addWidget(display_forces_button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setLayout(mainLayout);

  // Position and resize this widget
  move(10, 10);
  resize(120, 180);
}

GazeboViewControlPlugin::~GazeboViewControlPlugin() {
}

void GazeboViewControlPlugin::ForceCallback(ConstVector3dPtr& force_msg) {
  force_vector_ = math::Vector3(force_msg->x(), force_msg->y(), force_msg->z());
}

/*void GazeboViewControlPlugin::TorqueCallback(ConstVector3dPtr &torque_msg) {
  torque_vector_ = math::Vector3(torque_msg->x(), torque_msg->y(), torque_msg->z());
}*/

void GazeboViewControlPlugin::Load(sdf::ElementPtr _sdf) {
  // Get a pointer to the active user camera and the scene
  user_cam_ = gui::get_active_camera();

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(gui::get_world());
  this->force_sub_ = this->node_->Subscribe("~/fw_forces", &GazeboViewControlPlugin::ForceCallback, this);
  //this->torque_sub_ = this->node_->Subscribe("~/fw_torques",&GazeboViewControlPlugin::TorqueCallback, this);

#if GAZEBO_MAJOR_VERSION > 6
  cam_offset_ = user_cam_->WorldPosition();
#else
  cam_offset_ = user_cam_->GetWorldPosition();
#endif

  force_vector_ = math::Vector3::Zero;
  //torque_vector_ = math::Vector3::Zero;

  mode_ = "None";

  this->update_connection_ =
      event::Events::ConnectRender(
          boost::bind(&GazeboViewControlPlugin::OnUpdate, this));

  //gui::MouseEventHandler::Instance()->AddPressFilter(
  //            "rotors", boost::bind(&GazeboViewControlPlugin::OnMousePress, this, _1));
}

void GazeboViewControlPlugin::OnForwardButton() {
  is_tracking_ = true;
  cam_offset_ = kForwardCamOffset;
  mode_ = "Forward";
}

void GazeboViewControlPlugin::OnChaseButton() {
  is_tracking_ = true;
  cam_offset_ = kChaseCamOffset;
  mode_ = "Chase";
}

void GazeboViewControlPlugin::OnOrthogonalButton() {
  is_tracking_ = true;
  cam_offset_ = kOrthogonalCamOffset;
  cam_rotation_ = kOrthogonalRotation;
  mode_ = "Orthogonal";
}

void GazeboViewControlPlugin::OnCameraButton() {
  is_tracking_ = true;
  cam_offset_ = kCameraCamOffsetStart;
  offset_step_ = (kCameraCamOffsetGoal - kCameraCamOffsetStart) * 0.00005;
  cam_rotation_ = kOrthogonalRotation;
  rotation_step_ = (kCameraCamRotationGoal - kOrthogonalRotation) * 0.00004;
  num_steps_ = (kCameraCamRotationGoal.y - kOrthogonalRotation.y) / 0.00004;
  mode_ = "Camera";
}

void GazeboViewControlPlugin::OnStopButton() {
  is_tracking_ = false;
  mode_ = "None";
}

void GazeboViewControlPlugin::OnShowForces() {
  if (!display_forces_)
    display_forces_ = true;
  else {
    display_forces_ = false;
    rendering_force_->SetForce(math::Vector3(0.0, 0.0, 0.0));
  }
}

void GazeboViewControlPlugin::OnUpdate() {
  // If it has not been loaded already, attempt to get a handle to the visual
  // we want to track
  if (!visual_) {
    rendering::ScenePtr scene = rendering::get_scene();
    visual_ = scene->GetVisual("techpod::techpod/base_link");
    return;
  }

  if (!rendering_force_) {
    rendering_force_.reset(new RenderingForce("forces_visual", visual_));

    rendering_force_->Load();
    return;
  }

  /*if (!rendering_torque_) {
    rendering_torque_.reset(new RenderingTorque("torques_visual", visual_));

    rendering_torque_->Load();
  }*/

  if (display_forces_)
    rendering_force_->SetForce(force_vector_);

  //rendering_torque_->SetTorque(torque_vector_);

  if (is_tracking_) {
    // Get the world pose of the visual we are tracking
    math::Pose visual_pose = visual_->GetWorldPose();

    math::Pose new_cam_pose;
    math::Vector3 cam_offset_body = math::Vector3::Zero;

    // Align the camera with the orientation of the visual, unless it's
    // orthogal mode, then we point the camera at the visual
    if (mode_ == "Orthogonal") {
      cam_offset_body = visual_pose.rot.RotateVector(cam_offset_);

      math::Quaternion rot(cam_rotation_.x, cam_rotation_.y, cam_rotation_.z);

      new_cam_pose.rot = visual_pose.rot * rot;
    }
    else if (mode_ == "Camera") {
      double dist = (kCameraCamOffsetGoal - cam_offset_).GetLength();

      if (dist > 0.01)
        cam_offset_ += offset_step_;
      else {
        mode_ = "Wait";
        timer_start_ = common::Time::GetWallTime();
      }

      cam_offset_body = visual_pose.rot.RotateVector(cam_offset_);

      math::Quaternion rot(cam_rotation_.x, cam_rotation_.y, cam_rotation_.z);

      new_cam_pose.rot = visual_pose.rot * rot;
    }
    else if (mode_ == "Camera_Rotation") {
      double dist = (kCameraCamRotationGoal - cam_rotation_).GetLength();

      if (dist > 0.01 && num_steps_ > 0) {
        cam_offset_ += offset_step_;
        cam_rotation_ += rotation_step_;
        num_steps_--;
      }

      cam_offset_body = visual_pose.rot.RotateVector(cam_offset_);

      math::Quaternion rot(cam_rotation_.x, cam_rotation_.y, cam_rotation_.z);

      new_cam_pose.rot = visual_pose.rot * rot;
    }
    else if (mode_ == "Wait") {
      common::Time current_time = common::Time::GetWallTime();

      if ((current_time.Double() - timer_start_.Double()) > kWaitTime) {
        mode_ = "Camera_Rotation";
        offset_step_ = (kCameraCamOffsetSecondGoal - cam_offset_) / num_steps_;
      }

      cam_offset_body = visual_pose.rot.RotateVector(cam_offset_);

      math::Quaternion rot(cam_rotation_.x, cam_rotation_.y, cam_rotation_.z);

      new_cam_pose.rot = visual_pose.rot * rot;
    }
    else {
      new_cam_pose.rot.w = visual_pose.rot.w;
      new_cam_pose.rot.x = visual_pose.rot.x;
      new_cam_pose.rot.y = visual_pose.rot.y;
      new_cam_pose.rot.z = visual_pose.rot.z;

      // Rotate the camera position offset vector into the visual's body frame
      cam_offset_body = visual_pose.rot.RotateVector(cam_offset_);
    }

    // Translate the position of the camera
    new_cam_pose.pos.x = visual_pose.pos.x + cam_offset_body.x;
    new_cam_pose.pos.y = visual_pose.pos.y + cam_offset_body.y;
    new_cam_pose.pos.z = visual_pose.pos.z + cam_offset_body.z;

    // Set the new camera pose
    user_cam_->SetWorldPose(new_cam_pose);
  }
}

bool GazeboViewControlPlugin::OnMousePress(const common::MouseEvent& _event) {
  math::Vector3 world_pos;
  math::Vector2i click_pos = _event.Pos();
  rendering::ScenePtr scene = rendering::get_scene();

  scene->GetFirstContact(user_cam_, click_pos, world_pos);

  std::cout << world_pos.x << ", " << world_pos.y << ", " << world_pos.z << std::endl;

  return true;
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboViewControlPlugin);
}

