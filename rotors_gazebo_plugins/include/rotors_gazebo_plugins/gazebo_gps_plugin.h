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

#ifndef ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H

#include <gazebo/sensors/Noise.hh>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/gazebo_api_wrapper.hpp"

namespace gazebo {
// Default values
static const std::string kDefaultGroundSpeedPubTopic = "ground_speed";

class GazeboGpsPlugin : public SensorPlugin {
 public:
  GazeboGpsPlugin();
  virtual ~GazeboGpsPlugin();

 protected:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  void OnUpdate();

 private:
  ros::NodeHandle* node_handle_;
  ros::Publisher gps_pub_;
  ros::Publisher ground_speed_pub_;
  std::string namespace_;
  std::string gps_topic_;
  std::string ground_speed_topic_;
  std::string frame_id_;
  std::string link_name_;

  // Pointer to the parent sensor
  GazeboGpsSensorPtr parent_sensor_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  // GPS message to be published on sensor update
  sensor_msgs::NavSatFix gps_message_;

  // Ground speed message to be publised on sensor update
  geometry_msgs::Vector3Stamped ground_speed_message_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H
