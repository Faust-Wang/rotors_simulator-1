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

#ifndef ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// WGS84 constants
static constexpr double kEquatorialRadius = 6378137.0;
static constexpr double kPolarRadius = 6356752.3;

// Default reference values (Zurich: lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultRefLat = 47.3667;
static constexpr double kDefaultRefLon = 8.5500;
static constexpr double kDefaultRefAlt = 500.0;
static constexpr double kDefaultRefHeading = 0.0;

class GazeboGpsPlugin : public ModelPlugin {
 public:
  GazeboGpsPlugin();
  virtual ~GazeboGpsPlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string gps_topic_;
  ros::NodeHandle* node_handle_;
  ros::Publisher gps_pub_;
  std::string frame_id_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  int gps_sequence_;

  double ref_lat_;
  double ref_lon_;
  double ref_alt_;
  double ref_heading_;
  double earth_radius_;

  sensor_msgs::NavSatFix gps_message_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H
