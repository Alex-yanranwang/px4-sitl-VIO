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



#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"
#include "Wind.pb.h"

#include "common.h"

// USER HEADERS
#include <mav_msgs/Actuators.h>
#include <ros/ros.h>
#include <vector>


namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {

// Protobuf test
typedef const boost::shared_ptr<const std_msgs::msgs::Float> MotorSpeedPtr;
// static const std::string kDefaultMotorTestSubTopic = "motors";

class KinoMotorInterface : public ModelPlugin {
 public:
  // KinoMotorInterface()
  //     : ModelPlugin(){}

  // virtual ~KinoMotorInterface();

  // virtual void InitializeParams();
  
  // void testProto(MotorSpeedPtr &msg);
  void testProto(MotorSpeedPtr &msg);

 protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string motor_speed_sub_topic_;
  std::string namespace_;
  std::string link_name_;

  transport::NodePtr node_handle_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // // Protobuf test
  // std::string motor_test_sub_topic_;
  transport::SubscriberPtr motor_sub_;

  //ALEX Add for publish to ROS
  ros::NodeHandle ros_handle;
  ros::Publisher motor_ros_pub;
  std::string motor_speed_pub_topic_;
  std::string frame_id_;

  std::string motor_ros_topic_;
  mav_msgs::Actuators motor_ros_msg_;

};
}
