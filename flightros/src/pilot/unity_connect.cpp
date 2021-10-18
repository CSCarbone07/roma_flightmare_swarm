#include "flightros/pilot/unity_connect.hpp"


UnityConnect::UnityConnect()
{

// test comment


}

UnityConnect::~UnityConnect() {}



bool UnityConnect::setUnity() {



  unity_bridge_ptr_ = flightlib::UnityBridge::getInstance();
  //ROS_INFO("[%s] Unity Bridge is connected.", pnh_.getNamespace().c_str());
  ROS_INFO("Unity Bridge is set.");
 


  return true;
}


bool UnityConnect::connectUnity() {
  if (unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  //unity_ready_ = unity_bridge_ptr_->connectUnity(0);
  ROS_INFO("Unity Bridge is connected.");
  return unity_ready_;
}

void UnityConnect::addQuadrotor(std::shared_ptr<flightlib::Quadrotor> inQuad)
{
  quad_ptrs.push_back(inQuad);
  unity_bridge_ptr_->addQuadrotor(inQuad);
  ROS_INFO("Quadrotor added.");
}


void UnityConnect::updateUnity()
{
  unity_bridge_ptr_->getRender(0);      //update agents
  unity_bridge_ptr_->handleOutput();  //get imagery
  //ROS_INFO("Updating unity state.");
}

