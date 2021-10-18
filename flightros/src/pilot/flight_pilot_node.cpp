#include "flightros/pilot/flight_pilot_node.hpp"
#include "flightros/pilot/unity_connect.hpp"
#include <iostream>
#include <string>
#include <mutex>

#include "std_msgs/String.h"

#include <sstream>

//static std::shared_ptr<UnityBridge> unity_bridge_ref = UnityBridge::getInstance();

int main(int argc, char** argv) {

  ros::init(argc, argv, "flight_pilot");
  unsigned quad_count = 3;  
  bool useCameras = true;

  ros::NodeHandle n;
  //flightros::FlightPilot pilot(ros::NodeHandle(), ros::NodeHandle("~"));


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  //ros::NodeHandle nh_ = ros::NodeHandle();
  //ros::NodeHandle pnh_ = ros::NodeHandle("~");
/*  
  std::vector<flightros::FlightPilot*> pilots;
  for(unsigned i = 0; i<1; i++)
  {
    std::string secondNameSpace = "~";
    secondNameSpace.append(std::to_string(i+1));
    //flightros::FlightPilot pilot(nh_, pnh_);
    flightros::FlightPilot* pilot = new flightros::FlightPilot(ros::NodeHandle(), ros::NodeHandle("~"));

    std::string subTopic;
    subTopic.append("/uav_00");
    subTopic.append(std::to_string(i+1));
    subTopic.append("/hummingbird/ground_truth/odometry/");
    pilot->setPoseTopic(subTopic);
  
    pilot->setPilotId(i);

    pilot->setUnityRef();

    pilots.push_back(pilot);
  }
*/
  ros::Duration(5.0).sleep();

  // Initialize quadrotor
  std::vector<std::shared_ptr<Quadrotor>> quad_ptrs;// = std::make_shared<Quadrotor>();
  std::vector<QuadState*> quad_states;
  std::vector<flightros::FlightPilot*> flightPilots;

  UnityConnect* unityConnect = new UnityConnect();
  unityConnect->setUnity();

  for(unsigned i = 0; i<quad_count; i++)
  {

    std::shared_ptr<Quadrotor> newQuadrotor;
    newQuadrotor = std::make_shared<Quadrotor>();
    quad_ptrs.push_back(newQuadrotor);

    QuadState* new_quad_state = new QuadState();
    new_quad_state->setZero();
    quad_states.push_back(new_quad_state); 



    flightros::FlightPilot* newFlightPilot = new flightros::FlightPilot(ros::NodeHandle(), ros::NodeHandle("~"));
    newFlightPilot->setQuadrotor(newQuadrotor);
    //newFlightPilot->setCameras();

    std::string subTopic;
    std::string pubTopic;
    if(i<9)
    {
      subTopic.append("/uav_00");
      pubTopic.append("/uav_00");
    }
    else
    {
      subTopic.append("/uav_0");
      pubTopic.append("/uav_0");
    }
    subTopic.append(std::to_string(i+1));
    pubTopic.append(std::to_string(i+1));

    subTopic.append("/hummingbird/ground_truth/odometry/");
    pubTopic.append("/hummingbird/");

    newFlightPilot->setPoseTopic(subTopic);
    newFlightPilot->setCameraTopic(pubTopic);

    newFlightPilot->setPilotId(i);
    flightPilots.push_back(newFlightPilot);

    unityConnect->addQuadrotor(newQuadrotor);

  }

  unityConnect->connectUnity();

  for (flightros::FlightPilot* pilot : flightPilots)
  {
    pilot->setUnityRef();
    pilot->setUnityReady();
  }

/*
  // Initialize Unity bridge
  for (std::shared_ptr<Quadrotor> quad : quad_ptrs)
  {

  }
*/




  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    unityConnect->updateUnity();
    
    for (flightros::FlightPilot* pilot : flightPilots)
    {
      pilot->publishCameras();
    }
    
    //ROS_INFO("Ros before spinning");

/*
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    chatter_pub.publish(msg);
*/

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  } 




  //ROS_INFO("Flight pilot node has pilot 0 with frame %d id", pilots.at(0).pilotId_);
  //ROS_INFO("Flight pilot node has pilot 1 with frame %d id", pilots.at(1).pilotId_);
  //ROS_INFO("Flight pilot node started with name %s", ros::this_node::getName());

  // spin the ros
  //ros::spin();

  return 0;
}
