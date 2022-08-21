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


#include "kino_motor_interface.h"
#include <ignition/math.hh>

namespace gazebo {

// KinoMotorInterface::~KinoMotorInterface() {
//   updateConnection_->~Connection();
// }

// void KinoMotorInterface::InitializeParams() {}

void KinoMotorInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[kino_motor_plugin] Please specify a linkName.\n";
  frame_id_ = link_name_;
  
  getSdfParam<std::string>(_sdf, "motorSpeedSubTopic", motor_speed_sub_topic_,
                           motor_speed_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);


  // Protobuf test
  motor_sub_ = node_handle_->Subscribe<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_sub_topic_, 
                &KinoMotorInterface::testProto, this);
  //gzmsg <<"53 Motor_topic:"<<"~/"+model_->GetName() + motor_speed_sub_topic_<<"\n";

  this->motor_ros_pub = ros_handle.advertise<mav_msgs::Actuators>(motor_speed_pub_topic_,10);
  gzmsg <<"58 Motor_pub_topic:"<<motor_speed_pub_topic_<<"\n";

  std::vector<double> ang_v_init(4, 0.0);
  motor_ros_msg_.angular_velocities.clear();
  motor_ros_msg_.angular_velocities = ang_v_init;
  // motor_ros_msg_.angular_velocities.push_back(0.0);
  // motor_ros_msg_.angular_velocities.push_back(0.0);
  // motor_ros_msg_.angular_velocities.push_back(0.0);
  // motor_ros_msg_.angular_velocities.push_back(0.0);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&KinoMotorInterface::OnUpdate, this, _1));
}

// Protobuf test
void KinoMotorInterface::testProto(MotorSpeedPtr &msg) {
  //gzmsg <<"62 msg:"<<msg->data()<<"   motor_number:"<<msg->motor_number()<<"\n";
  
  int motorNumber_ = msg->motor_number();
  double motorSpeed_ = msg->data();

  //gzmsg <<"67 msg:"<<motorSpeed_<<"   motor_number:"<<motorNumber_<<"\n";

  switch (motorNumber_)
  {
    case 0:
      motor_ros_msg_.angular_velocities[0] = motorSpeed_;
      break;
    case 1:
      motor_ros_msg_.angular_velocities[1] = motorSpeed_;
      break;
    case 2:
      motor_ros_msg_.angular_velocities[2] = motorSpeed_;
      break;
    case 3:
      motor_ros_msg_.angular_velocities[3] = motorSpeed_;
      break;
  }

  common::Time current_time  = world_->SimTime();

  motor_ros_msg_.header.frame_id = frame_id_;
  motor_ros_msg_.header.stamp.sec = current_time.sec;
  motor_ros_msg_.header.stamp.nsec = current_time.nsec;

  motor_ros_pub.publish(motor_ros_msg_);
}


// This gets called by the world update start event.
void KinoMotorInterface::OnUpdate(const common::UpdateInfo& _info) {
  
}

GZ_REGISTER_MODEL_PLUGIN(KinoMotorInterface);
}
