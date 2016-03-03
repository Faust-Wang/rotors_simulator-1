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

#ifndef ROTORS_GAZEBO_PLUGINS_WING_MODELS_H
#define ROTORS_GAZEBO_PLUGINS_WING_MODELS_H

#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

class GazeboWingModelPlugin : public ModelPlugin {
 public:
  GazeboWingModelPlugin();
  virtual ~GazeboWingModelPlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

  math::Vector3 ComputeAerodynamicForces(math::Vector3 vel);
  //math::Vector3 get_aerodynamic_moments(math::Vector3 &vel);

 private:
  std::string namespace_;
  //std::string pose_topic_;
  ros::NodeHandle *node_handle_;
  //ros::Subscriber pose_sub_;
  std::string frame_id_;

  /// \brief Pointer to the world.
  physics::WorldPtr world_;
  /// \brief Pointer to the model.
  physics::ModelPtr model_;
  /// \brief Pointer to the link.
  physics::LinkPtr link_;
  /// \brief The connections.
  event::ConnectionPtr updateConnection_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_WING_MODELS_H
