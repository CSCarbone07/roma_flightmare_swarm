
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



using namespace flightlib;

namespace flightros {

class FlightPilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightPilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);//, int initialSleep);
  ~FlightPilot();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);


  void setPoseTopic(std::string inTopic);
  void setCameraTopic(std::string inTopic);

  void setUnityRef();//(std::shared_ptr<UnityBridge> inREF);
  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

  void setPilotId(int in_id);
  int pilotId_ = 0;

  void setQuadrotor(std::shared_ptr<Quadrotor> inQuad);
  void setCameras();
  void setUnityReady();

  void publishCameras();

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool useCameras_ = true;
 
  bool isReadyToListen_pose_ = false; 
  bool isReadyToPublish_camera_ = false; 

  //std::mutex mutex_;
  //RemoteMutex mutex("nameOfMutex");


  // publisher
  image_transport::Publisher rgb_pub;
  image_transport::Publisher depth_pub;
  image_transport::Publisher segmentation_pub;
  image_transport::Publisher opticalflow_pub;

  // subscriber
  ros::Subscriber sub_state_est_;

  // main loop timer
  ros::Timer timer_main_loop_;

  ros::Time t0;
  

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  // trajectories
  polynomial_trajectories::PolynomialTrajectory trajectory;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliary variables
  Scalar main_loop_freq_{50.0};

  std::string subscribePoseTopic;
  std::string publishCameraTopic;

  //test
  std::string node_namespace;
  std::string node_test = "/uav_002/hummingbird/flight_pilot_node";


};
}  // namespace flightros
