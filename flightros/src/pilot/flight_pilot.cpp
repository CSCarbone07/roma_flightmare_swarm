#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)//, int initialSleep)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }




  // quad initialization
  //quad_ptr_ = std::make_shared<Quadrotor>();

  // initialization
  quad_state_.setZero();
  //quad_ptr_->reset(quad_state_);

  // set trajectory
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(0, 10, 2.5));
  way_points.push_back(Eigen::Vector3d(5, 0, 2.5));
  way_points.push_back(Eigen::Vector3d(0, -10, 2.5));
  way_points.push_back(Eigen::Vector3d(-5, 0, 2.5));

/*
  std::size_t num_waypoints = way_points.size();
  Eigen::VectorXd segment_times(num_waypoints);
  segment_times << 10.0, 10.0, 10.0, 10.0;
  Eigen::VectorXd minimization_weights(num_waypoints);
  minimization_weights << 1.0, 1.0, 1.0, 1.0;

  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
    polynomial_trajectories::PolynomialTrajectorySettings(
      way_points, minimization_weights, 7, 4);

  trajectory = polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,20.0, 20.0, 6.0);
*/
  
  //ros::Time t0 = ros::Time::now();

  // initialize subscriber call backs
  //sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 //&FlightPilot::poseCallback, this);
  //sub_state_est_ = nh_.subscribe("/uav_001/hummingbird/flight_pilot/state_estimate", 1,
    //                             &FlightPilot::poseCallback, this);

  //timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
    //                                 &FlightPilot::mainLoopCallback, this);




  node_namespace = pnh_.getNamespace().c_str();




  // wait until the gazebo and unity are loaded
  //ros::Duration(5.0).sleep();
  //this->unity_render_ = unity_render_;
  // connect unity

  //setUnity(unity_render_);
  //connectUnity();

}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  //ROS_INFO("Flight pilot node for frame %d has quadstate posy %f ", pilotId_, quad_state_.x[QS::POSY]);
  //ROS_INFO("Flight pilot node %d getting quadstate posy %f ", pilotId_, msg->pose.pose.position.y);

  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

/*
  //ROS_INFO("Flight pilot node receiving posecallback with x %d ", (Scalar)msg->pose.pose.position.x);
  //ROS_INFO("Flight pilot node setting posecallback with x %d ", quad_state_.x[QS::POSX]);
  ROS_INFO("Flight pilot node %d getting quadstate posx %f ", pilotId_, msg->pose.pose.position.x);
  ROS_INFO("Flight pilot node %d getting quadstate posy %f ", pilotId_, msg->pose.pose.position.y);
  ROS_INFO("Flight pilot node %d getting quadstate posz %f ", pilotId_, msg->pose.pose.position.z);
  ROS_INFO("Flight pilot node %d setting quadstate posx %f ", pilotId_, quad_state_.x[QS::POSX]);
  ROS_INFO("Flight pilot node %d setting quadstate posy %f ", pilotId_, quad_state_.x[QS::POSY]);
  ROS_INFO("Flight pilot node %d setting quadstate posz %f ", pilotId_, quad_state_.x[QS::POSZ]);
  //ROS_INFO("Flight pilot node setting quadstate posx %s ", typeid(quad_state_.x[QS::POSX]).name());
  //ROS_INFO("Flight pilot node setting quadstate posy %s ", typeid(quad_state_.x[QS::POSY]).name());
  //ROS_INFO("Flight pilot node setting quadstate posz %s ", typeid(quad_state_.x[QS::POSZ]).name());
*/

/*
  quadrotor_common::TrajectoryPoint desired_pose =
    polynomial_trajectories::getPointFromTrajectory(
      trajectory, ros::Duration(ros::Time::now() - t0));

  // Set pose
  quad_state_.x[QS::POSX] = (Scalar)desired_pose.position.x();
  quad_state_.x[QS::POSY] = (Scalar)desired_pose.position.y();
  quad_state_.x[QS::POSZ] = (Scalar)desired_pose.position.z();
  quad_state_.x[QS::ATTW] = (Scalar)desired_pose.orientation.w();
  quad_state_.x[QS::ATTX] = (Scalar)desired_pose.orientation.x();
  quad_state_.x[QS::ATTY] = (Scalar)desired_pose.orientation.y();
  quad_state_.x[QS::ATTZ] = (Scalar)desired_pose.orientation.z();
*/

  //

  if(isReadyToListen_pose_)
  {
   quad_ptr_->setState(quad_state_);
  }
   

/*
  if (unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();
  }
*/
/*
  if (unity_render_ && unity_ready_) {

  }
*/
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

void FlightPilot::setPilotId(int in_id)
{
  ROS_INFO("Flight pilot node getting %d as frame id", in_id);
  pilotId_ = in_id;
  ROS_INFO("Flight pilot node setting %d as frame id", pilotId_);
}

void FlightPilot::setUnityRef(){//(std::shared_ptr<UnityBridge> inREF) {
  //unity_bridge_ptr_ = inREF;
  unity_bridge_ptr_ = UnityBridge::getInstance();
  //setUnity(unity_render_);
}

void FlightPilot::setPoseTopic(std::string inTopic) 
{
  subscribePoseTopic= inTopic;
  sub_state_est_ = nh_.subscribe(subscribePoseTopic, 1,
                                 &FlightPilot::poseCallback, this);

  isReadyToListen_pose_ = true; 
}

void FlightPilot::setCameraTopic(std::string inTopic) 
{
  publishCameraTopic = inTopic;

  std::string rgb_topic{inTopic};
  std::string dep_topic{inTopic};
  std::string seg_topic{inTopic};
  std::string opt_topic{inTopic};



  rgb_topic.append("rgb");
  dep_topic.append("depth");
  seg_topic.append("segmentation");
  opt_topic.append("opticalflow");


  std::cout << "Flight pilot setting " << inTopic << " as camera namespace" << std::endl;
  std::cout << "Flight pilot setting " << rgb_topic << " as rgb camera topic" << std::endl;
/*
  ROS_INFO("Flight pilot setting %s as camera publish topic", inTopic);
  ROS_INFO("Flight pilot setting %s as camera publish topic", rgb_topic);
  ROS_INFO("Flight pilot setting %s as camera publish topic", dep_topic);
  ROS_INFO("Flight pilot setting %s as camera publish topic", seg_topic);
  ROS_INFO("Flight pilot setting %s as camera publish topic", opt_topic);
*/

  // initialize publishers
  image_transport::ImageTransport it(pnh_);
  rgb_pub = it.advertise(rgb_topic, 1);
  depth_pub = it.advertise(dep_topic, 1);
  segmentation_pub = it.advertise(seg_topic, 1);
  opticalflow_pub = it.advertise(opt_topic, 1);

  isReadyToPublish_camera_ = true; 
}

bool FlightPilot::setUnity(const bool render) {

  unity_render_ = render;
  if (unity_render_) 
  {
    // create unity bridge


    //unity_bridge_ptr_ = UnityBridge::getInstanceNonStatic();   
    /*
    if(node_namespace == node_test)
    { 
    }
    */

    //mutex_.lock();
    //unity_bridge_ptr_ = UnityBridge::getInstanceNonShared();
    //mutex_.unlock();
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    //ROS_INFO("[%s] Unity Bridge is connected.", pnh_.getNamespace().c_str());
  }
  connectUnity();
  

  return true;
}


bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

void FlightPilot::setQuadrotor(std::shared_ptr<Quadrotor> inQuad) 
{
  quad_ptr_ = inQuad;

  std::cout << "Flight pilot quad set" << std::endl;

  if(useCameras_)
  {setCameras();}

}

void FlightPilot::setCameras()
{

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, -0.3);
  //x+ = up, y+ = front, z = right side
  //Matrix<3, 3> R_BC = Quaternion(0.0, 0.0, 1.0, -1.0*3.14159265359).toRotationMatrix();
  // Default rotation (looking south)
  //Matrix<3, 3> R_BC = Quaternion(0.0, 0.0, -0.0, 1.0).toRotationMatrix();
  // Rotation to look forward in the world
  //Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  // Rotation to look forward down in the world
  Matrix<3, 3> R_BC = Quaternion(0.7071068, -0.7071068, 0, 0 ).toRotationMatrix();
  //Matrix<3, 3> R_BC = Quaternion(0, 0.6335811, -0.6335811, 0.4440158 ).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(84.54737801); //90
  rgb_camera_->setWidth(1600); //720
  rgb_camera_->setHeight(1200); //480
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  std::cout << "Flight pilot camera set" << std::endl;

}



void FlightPilot::setUnityReady()
{
unity_ready_ = true;
}


void FlightPilot::publishCameras()
{
  if(useCameras_ && unity_ready_ && isReadyToPublish_camera_)
  {
    // camera
    cv::Mat img;

    ros::Time timestamp = ros::Time::now();

    rgb_camera_->getRGBImage(img);

    
    //std::cout << img << std::endl;
    sensor_msgs::ImagePtr rgb_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);
/*
    rgb_camera_->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
    depth_msg->header.stamp = timestamp;
    depth_pub.publish(depth_msg);

    rgb_camera_->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp = timestamp;
    segmentation_pub.publish(segmentation_msg);
*/


  }
}





}  // namespace flightros
