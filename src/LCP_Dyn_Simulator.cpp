#include <lcp_val_simulator/LCP_Dyn_Simulator.hpp>

LCP_Dyn_Simulator::LCP_Dyn_Simulator():m_q(NUM_Q), m_qdot(NUM_QDOT),
                                     m_tau(NUM_QDOT), cori_(NUM_QDOT),
                                     grav_(NUM_QDOT), A_(NUM_QDOT, NUM_QDOT){
  m_q.setZero();  
  m_qdot.setZero();  
  m_tau.setZero();
  cori_.setZero(); 
  grav_.setZero(); 
  A_.setZero();
  
  robot_model_ = RobotModel::GetRobotModel();

  m_sim_rate = 1/1000.0;

  Initialize_Simulator();

  printf("[Valkyrie Dynamic Control Test] Constructed\n");
}

LCP_Dyn_Simulator::~LCP_Dyn_Simulator(){}

void LCP_Dyn_Simulator::Initialize_Simulator(){
  // Initialize to Valkyrie Standing Up
	  // Set Virtual Joints
	  // x_pos
	  m_q[0] = 0.0;
	  // y_pos
	  m_q[1] = 0.0;
	  // z_pos
	  m_q[2] = 1.131; 
}

void LCP_Dyn_Simulator::UpdateModel(){
	robot_model_->UpdateModel(m_q, m_qdot);
	robot_model_->getMassInertia(A_);	
	robot_model_->getGravity(grav_);	
	robot_model_->getCoriolis(cori_);	
}


void LCP_Dyn_Simulator::MakeOneStepUpdate(){
	// Update Model
	UpdateModel();

	// Get Torque Command

	// Perform Time Integratation	
	//qddot_next = Ainv*(tau - b - g)

	// Virtual Sphere Joints
	//qdot_next = qddot*dt + qdot_next
	//q_next = quatmultiply(qdot_next*dt, q)  

	std::cout << "[LCP Dyn Simulator] Step Update" << std::endl;

}