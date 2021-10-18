
#pragma once

#include <memory>
#include <mutex>
//#include "remote_mutex/RemoteMutex.h"

// ros
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// rpg quadrotor
#include <autopilot/autopilot_helper.h>
#include <autopilot/autopilot_states.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>



class UnityConnect {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  UnityConnect();
  ~UnityConnect();


  bool setUnity();
  bool connectUnity();
  void addQuadrotor(std::shared_ptr<flightlib::Quadrotor> inQuad);

  void updateUnity();

 private:
 
  bool unity_ready_{false};
  std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;

  std::vector<std::shared_ptr<flightlib::Quadrotor>> quad_ptrs;

  flightlib::SceneID scene_id_{flightlib::UnityScene::WAREHOUSE};

  //test
  std::string node_namespace;
  std::string node_test = "/uav_002/hummingbird/flight_pilot_node";


};


