#include "ros/ros.h"

#include <lcp_val_simulator/LCP_Dyn_Simulator.hpp>

#include "valkyrie_definition.h"
#include "RobotModel.hpp"
/*#include "Utils/utilities.hpp"
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include <Valkyrie/Robot_Model/Valkyrie_Left_Leg.hpp>
#include <Valkyrie/Robot_Model/Valkyrie_Right_Leg.hpp>*/

int main(int argc, char **argv){
  ros::init(argc, argv, "lcp_val_simulator_node");
  ros::NodeHandle n;

  LCP_Dyn_Simulator lcp_simulator;


  ros::Rate loop_rate(10);

  std::cout << "Simulator Running" << std::endl;
  while (ros::ok()){
    lcp_simulator.MakeOneStepUpdate();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}