#include <lcp_val_simulator/lcp_dyn_simulator.hpp>
#include <Optimizer/SJMobyLCP/MobyLCP.h>

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


void LCP_Dyn_Simulator::CreateFootContactModel(){
  // lf - left foot, rf - right foot
  // OF - Out Front
  // OB - Out Back
  // IB - In Back
  // IF - In Front

  // Left Foot Contact
  sejong::Vect3 lf_OF_pos;
  sejong::Vect3 lf_OB_pos;
  sejong::Vect3 lf_IB_pos;  
  sejong::Vect3 lf_IF_pos;  

  sejong::Matrix J_lf_OF(6,NUM_QDOT);
  sejong::Matrix J_lf_OB(6,NUM_QDOT);
  sejong::Matrix J_lf_IB(6,NUM_QDOT);
  sejong::Matrix J_lf_IF(6,NUM_QDOT);

  sejong::Matrix J_lf_OF_c(1,NUM_QDOT);  
  sejong::Matrix J_lf_OB_c(1,NUM_QDOT);    
  sejong::Matrix J_lf_IB_c(1,NUM_QDOT);
  sejong::Matrix J_lf_IF_c(1,NUM_QDOT);

  // Get the Positions of the Left Foot contacts
  robot_model_->getPosition(m_q, SJLinkID::LK_leftFootOutFront, lf_OF_pos);
  robot_model_->getPosition(m_q, SJLinkID::LK_leftFootOutBack, lf_OB_pos);
  robot_model_->getPosition(m_q, SJLinkID::LK_leftFootInBack, lf_IB_pos);
  robot_model_->getPosition(m_q, SJLinkID::LK_leftFootInFront, lf_IF_pos);

  // Get the Jacobian of the Z direction
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_leftFootOutFront, J_lf_OF);
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_leftFootOutBack, J_lf_OB);
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_leftFootInBack, J_lf_IB);  
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_leftFootInFront, J_lf_IF);  

  J_lf_OF_c.block(0,0, 1, NUM_QDOT) = J_lf_OF.block(5, 0, 1, NUM_QDOT); // Z
  J_lf_OB_c.block(0,0, 1, NUM_QDOT) = J_lf_OB.block(5, 0, 1, NUM_QDOT); // Z  
  J_lf_IB_c.block(0,0, 1, NUM_QDOT) = J_lf_IB.block(5, 0, 1, NUM_QDOT); // Z  
  J_lf_IF_c.block(0,0, 1, NUM_QDOT) = J_lf_IF.block(5, 0, 1, NUM_QDOT); // Z      

  //sejong::pretty_print(lf_OF_pos, std::cout, "Left Foot Out Front Position");
  sejong::pretty_print(J_lf_OF_c, std::cout, "Jacobian lf_OF contact");  
  sejong::pretty_print(J_lf_OB_c, std::cout, "Jacobian lf_OB contact");  
  sejong::pretty_print(J_lf_IB_c, std::cout, "Jacobian lf_IB contact");  
  sejong::pretty_print(J_lf_IF_c, std::cout, "Jacobian lf_IF contact");      


  // Contact Phi Jacobian
  sejong::Matrix J_phi(4,NUM_QDOT);  
  sejong::Vector phi(4); 

  J_phi.block(0,0, 1, NUM_QDOT) = J_lf_OF_c; // Jacobian of distance to contact point 1
  J_phi.block(1,0, 1, NUM_QDOT) = J_lf_OB_c; // Jacobian of distance to contact point 2
  J_phi.block(2,0, 1, NUM_QDOT) = J_lf_IB_c; // Jacobian of distance to contact point 3
  J_phi.block(3,0, 1, NUM_QDOT) = J_lf_IF_c; // Jacobian of distance to contact point 4

  phi[0] = lf_OF_pos[2]; // Distance to contact point 1
  phi[1] = lf_OB_pos[2]; // Distance to contact point 2
  phi[2] = lf_IB_pos[2]; // Distance to contact point 3
  phi[3] = lf_IB_pos[2]; // Distance to contact point 4  

  // Begin Friction Contact Modeling
  const int p = 4; // Number of Contacts
  const int d = 4; // Number of Friction Basis Vectors per Friction Cone

  double mu_static = 0.8;//0.1; // Friction Coefficient
  sejong::Vector n1(3); n1[2] = 1.0; // Normal Vector for contact 1
  sejong::Vector n2 = n1; // Normal Vector for contact 2
  sejong::Vector n3 = n1; // Normal Vector for contact 3
  sejong::Vector n4 = n1; // Normal Vector for contact 4 

  sejong::Matrix J_c1 = J_lf_OF.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 1
  sejong::Matrix J_c2 = J_lf_OB.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 2  
  sejong::Matrix J_c3 = J_lf_IB.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 1
  sejong::Matrix J_c4 = J_lf_IF.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 2    

  // Set Projected Normal Forces Direction
  sejong::Matrix N(NUM_QDOT, p);
  N.block(0,0, NUM_QDOT, 1) = J_c1.transpose()*n1;
  N.block(0,1, NUM_QDOT, 1) = J_c2.transpose()*n2; 
  N.block(0,2, NUM_QDOT, 1) = J_c3.transpose()*n3;
  N.block(0,3, NUM_QDOT, 1) = J_c4.transpose()*n4;  

  // Set Basis Vectors of Friction Cone 
  sejong::Matrix D1(3, d); // Basis for contact 1 
  D1.setZero();
  D1(0,0) = 1; 
  D1(1,1) = 1;   
  D1(0,2) = -1;
  D1(1,3) = -1;
  sejong::Matrix D2 = D1; // Basis for contact 2
  sejong::Matrix D3 = D1; // Basis for contact 3
  sejong::Matrix D4 = D1; // Basis for contact 4    


}

void LCP_Dyn_Simulator::MakeOneStepUpdate(){
	// Update Model
	UpdateModel();
	robot_model_->getInverseMassInertia(Ainv_);

	CreateFootContactModel();

	// Get Torque Command
//	m_tau[3] = 10.0;

	// Fix xyz in the air
/*	m_tau[0] = 200. * (0.0 - m_q[0]) + 20.*(-m_qdot[0]);
	m_tau[1] = 200. * (0.0 - m_q[1]) + 20.*(-m_qdot[1]);
  	m_tau[2] = 2000. * (1.131 - m_q[2]) + 20.*(-m_qdot[2]);
*/

	// Perform Time Integratation ---------------------------	
	double dt = m_sim_rate;
	sejong::Vector qddot_next = Ainv_*(m_tau - cori_ - grav_);
	sejong::Vector qdot_next = qddot_next*dt + m_qdot; 

	// Perform Next State Integration
	sejong::Vector q_next = m_q;

	// Time Integrate Linear Virtual Joints
	q_next.head(3) = qdot_next.head(3)*dt + m_q.head(3);

	// Time Integrate Virtual Sphere Joints
		// Get the Pelvis angular velocity
		sejong::Vect3 pelvis_omega;
		pelvis_omega[0] = m_qdot[3];
		pelvis_omega[1] = m_qdot[4];	
		pelvis_omega[2] = m_qdot[5];

		sejong::Quaternion quat_pelvis_current(m_q[NUM_Q-1], m_q[3], m_q[4], m_q[5]); 	// w, x, y, z
		sejong::Quaternion quat_world_rotate;
		sejong::convert(pelvis_omega*dt, quat_world_rotate);

		// Perform Extrinsic Quaternion Multiplication
		sejong::Quaternion quat_result = sejong::QuatMultiply(quat_world_rotate, quat_pelvis_current, true);  

		// The new virtual joint quaternion
		q_next[3] = quat_result.x();	
		q_next[4] = quat_result.y();	
		q_next[5] = quat_result.z();	
		q_next[NUM_Q-1] = quat_result.w();

	// Time Integrate Non Virtual Joints
	q_next.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL) = qdot_next.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL)*dt + m_q.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL) ;


	m_qdot = qdot_next;
	m_q = q_next;	

/*	std::cout << "m_q" << m_q << std::endl;
	std::cout << "m_q.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL)" << m_q.segment(NUM_VIRTUAL, NUM_QDOT-NUM_VIRTUAL) << std::endl;	 	
*/
	std::cout << "[LCP Dyn Simulator] Step Update" << std::endl;




}