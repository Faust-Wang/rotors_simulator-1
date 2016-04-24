/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_SERVO_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_SERVO_PLUGIN_H

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/Actuators.h>
#include <ros/ros.h>

#include "rotors_gazebo_plugins/common.h"

namespace positive_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
// Default values
static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";
static constexpr double kDefaultGain = 1.0;
static constexpr double kDefaultMinAngle = -M_PI * 0.25;
static constexpr double kDefaultMaxAngle = M_PI * 0.25;
static constexpr int kDefaultChannel = 0;

class GazeboServoPlugin : public ModelPlugin {
 public:

  GazeboServoPlugin();
  ~GazeboServoPlugin();

  void AngleCallback(const mav_msgs::ActuatorsConstPtr& servo_angles);

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the joint
  physics::JointPtr joint_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  double ref_angle_;
  double min_angle_;
  double max_angle_;

  double gain_;
  double damping_;

  int channel_;
  int positive_direction_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_SERVO_PLUGIN_H
