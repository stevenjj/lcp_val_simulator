#include <lcp_val_simulator/LCP_Dyn_Simulator.hpp>


LCP_Dyn_Simulator::LCP_Dyn_Simulator():m_q(NUM_Q), m_qdot(NUM_QDOT),
                                     m_tau(NUM_QDOT), cori_(NUM_QDOT),
                                     grav_(NUM_QDOT), A_(NUM_QDOT, NUM_QDOT), Ainv_(NUM_QDOT, NUM_QDOT){
  m_q.setZero();  
  m_qdot.setZero();  
  m_tau.setZero();
  cori_.setZero(); 
  grav_.setZero(); 
  A_.setZero();
  Ainv_.setZero();  
  
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

	
	  m_q[NUM_Q - 1] = 1.0; 	  


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
	robot_model_->getInverseMassInertia(Ainv_);

	// Get Torque Command

	// Perform Time Integratation	
	double dt = m_sim_rate;
	sejong::Vector qddot_next = Ainv_*(m_tau - cori_ - grav_);
	sejong::Vector qdot_next = qddot_next*m_sim_rate + m_qdot; 

	// Perform Next State Integration
	sejong::Vector q_next = m_q;

	// Time Integrate Linear Virtual Joints
	q_next.head(3) = qdot_next.head(3)*m_sim_rate + m_q.head(3);

	// Time Integrate Virtual Sphere Joints
	
	// Get the Pelvis angular velocity
	sejong::Vect3 pelvis_omega;
	pelvis_omega[0] = m_qdot[3];
	pelvis_omega[1] = m_qdot[4];	
	pelvis_omega[2] = m_qdot[5];

	sejong::Quaternion quat_pelvis_current(m_q[NUM_Q-1], m_q[3], m_q[4], m_q[5]); 	// w, x, y, z
	sejong::Quaternion quat_world_rotate;
	sejong::convert(pelvis_omega*m_sim_rate, quat_world_rotate);

	// Perform Extrinsic Quaternion Multiplication
	sejong::Quaternion quat_result = sejong::QuatMultiply(quat_world_rotate, quat_pelvis_current, true);  

	// The new virtual joint quaternion
	q_next[3] = quat_result.x();	
	q_next[4] = quat_result.y();	
	q_next[5] = quat_result.z();	
	q_next[NUM_Q-1] = quat_result.w();

	// Time Integrate Non Virtual Joints
	q_next.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL) = qdot_next.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL)*m_sim_rate + m_q.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL) ;


	std::cout << "m_qdot.head(NUM_VIRTUAL)" << m_qdot.head(NUM_VIRTUAL) << std::endl;

/*	std::cout << "pelvis_omega" << pelvis_omega << std::endl;	 


	std::cout << "m_q" << m_q << std::endl;
	std::cout << "m_q.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL)" << m_q.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL) << std::endl;	 	*/


	m_qdot = qdot_next;
	m_q = q_next;	
	std::cout << "[LCP Dyn Simulator] Step Update" << std::endl;




}