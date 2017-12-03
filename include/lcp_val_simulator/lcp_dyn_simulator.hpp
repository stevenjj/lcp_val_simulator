#ifndef LCP_DYN_SIM_H
#define LCP_DYN_SIM_H

#include "valkyrie_definition.h"
#include "RobotModel.hpp"
/*#include "Utils/utilities.hpp"
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include <Valkyrie/Robot_Model/Valkyrie_Left_Leg.hpp>
#include <Valkyrie/Robot_Model/Valkyrie_Right_Leg.hpp>*/

class LCP_Dyn_Simulator{
public:
  RobotModel* robot_model_;	

  void Initialize_Simulator();

  void UpdateModel();
  void MakeOneStepUpdate();

  double m_sim_rate;

  sejong::Vector m_q;
  sejong::Vector m_qdot;
  sejong::Vector m_tau;

  sejong::Vector cori_;
  sejong::Vector grav_;
  sejong::Matrix A_;
  sejong::Matrix Ainv_;


  LCP_Dyn_Simulator();
  ~LCP_Dyn_Simulator();    
};

#endif