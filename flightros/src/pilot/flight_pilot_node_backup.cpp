#include "flightros/pilot/flight_pilot_node.hpp"
#include<iostream>
#include <string>


//static std::shared_ptr<UnityBridge> unity_bridge_ref = UnityBridge::getInstance();

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_pilot");
  
  std::vector<flightros::FlightPilot> pilots;
  for(unsigned i = 0; i<2; i++)
  {
    std::string secondNameSpace = "~";
    secondNameSpace.append(std::to_string(i+1));
    flightros::FlightPilot pilot(ros::NodeHandle(), ros::NodeHandle("~"));
    
    std::string subTopic;
    subTopic.append("/uav_00");
    subTopic.append(std::to_string(i+1));
    subTopic.append("/hummingbird/ground_truth/odometry/");
    pilot.setPoseTopic(subTopic);
  
    pilot.setPilotId(i);

    pilot.setUnityRef();

    pilots.push_back(pilot);
  }
  
  //ROS_INFO("Flight pilot node has pilot 0 with frame %d id", pilots.at(0).pilotId_);
  //ROS_INFO("Flight pilot node has pilot 1 with frame %d id", pilots.at(1).pilotId_);



  //std::shared_ptr<flightlib::UnityBridge> unity_bridge_ref;// = UnityBridge::getInstance();
  //unity_bridge_ref = flightlib::UnityBridge::getInstance();  
  //pilot.setUnityRef(UnityBridge::getInstance());
  //pilot


  //ROS_INFO("Flight pilot node started with name %s", ros::this_node::getName());

  // spin the ros
  ros::spin();

  return 0;
}
