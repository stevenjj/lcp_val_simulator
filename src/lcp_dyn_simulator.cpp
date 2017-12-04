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
	  m_q[2] = 1.2; //1.131; 

	  m_q[NUM_Q - 1] = 1.0; 	  
}

void LCP_Dyn_Simulator::UpdateModel(){
	robot_model_->UpdateModel(m_q, m_qdot);
	robot_model_->getMassInertia(A_);	
	robot_model_->getGravity(grav_);	
	robot_model_->getCoriolis(cori_);	
}



void LCP_Dyn_Simulator::CreateFootContactModel(sejong::Matrix &N_mat, sejong::Matrix &B_mat,
											   sejong::Vector &fn_out, sejong::Vector &fd_out){
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

  // Right Foot Contact
  sejong::Vect3 rf_OF_pos;
  sejong::Vect3 rf_OB_pos;
  sejong::Vect3 rf_IB_pos;  
  sejong::Vect3 rf_IF_pos;  

  sejong::Matrix J_rf_OF(6,NUM_QDOT);
  sejong::Matrix J_rf_OB(6,NUM_QDOT);
  sejong::Matrix J_rf_IB(6,NUM_QDOT);
  sejong::Matrix J_rf_IF(6,NUM_QDOT);

  sejong::Matrix J_rf_OF_c(1,NUM_QDOT);  
  sejong::Matrix J_rf_OB_c(1,NUM_QDOT);    
  sejong::Matrix J_rf_IB_c(1,NUM_QDOT);
  sejong::Matrix J_rf_IF_c(1,NUM_QDOT);

  // Get the Positions of the Right Foot contacts
  robot_model_->getPosition(m_q, SJLinkID::LK_rightFootOutFront, rf_OF_pos);
  robot_model_->getPosition(m_q, SJLinkID::LK_rightFootOutBack, rf_OB_pos);
  robot_model_->getPosition(m_q, SJLinkID::LK_rightFootInBack, rf_IB_pos);
  robot_model_->getPosition(m_q, SJLinkID::LK_rightFootInFront, rf_IF_pos);

  // Get the Jacobian of the Z direction
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_rightFootOutFront, J_rf_OF);
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_rightFootOutBack, J_rf_OB);
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_rightFootInBack, J_rf_IB);  
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_rightFootInFront, J_rf_IF);  

  J_rf_OF_c.block(0,0, 1, NUM_QDOT) = J_rf_OF.block(5, 0, 1, NUM_QDOT); // Z
  J_rf_OB_c.block(0,0, 1, NUM_QDOT) = J_rf_OB.block(5, 0, 1, NUM_QDOT); // Z  
  J_rf_IB_c.block(0,0, 1, NUM_QDOT) = J_rf_IB.block(5, 0, 1, NUM_QDOT); // Z  
  J_rf_IF_c.block(0,0, 1, NUM_QDOT) = J_rf_IF.block(5, 0, 1, NUM_QDOT); // Z      

  //sejong::pretty_print(lf_OF_pos, std::cout, "Left Foot Out Front Position");
/*  sejong::pretty_print(J_lf_OF_c, std::cout, "Jacobian lf_OF contact");  
  sejong::pretty_print(J_lf_OB_c, std::cout, "Jacobian lf_OB contact");  
  sejong::pretty_print(J_lf_IB_c, std::cout, "Jacobian lf_IB contact");  
  sejong::pretty_print(J_lf_IF_c, std::cout, "Jacobian lf_IF contact");  */    


  // Begin Friction Contact Modeling
  const int p = 8; // Number of Contacts
  const int d = 4; // Number of Friction Basis Vectors per Friction Cone
  double mu_static = 0.8;//0.1; // Friction Coefficient


  // Contact Phi Jacobian
  sejong::Matrix J_phi(p,NUM_QDOT);  
  sejong::Vector phi(p); 

  // Left Foot
  J_phi.block(0,0, 1, NUM_QDOT) = J_lf_OF_c; // Jacobian of distance to contact point 1
  J_phi.block(1,0, 1, NUM_QDOT) = J_lf_OB_c; // Jacobian of distance to contact point 2
  J_phi.block(2,0, 1, NUM_QDOT) = J_lf_IB_c; // Jacobian of distance to contact point 3
  J_phi.block(3,0, 1, NUM_QDOT) = J_lf_IF_c; // Jacobian of distance to contact point 4

  // Right Foot
  J_phi.block(4,0, 1, NUM_QDOT) = J_rf_OF_c; // Jacobian of distance to contact point 5  
  J_phi.block(5,0, 1, NUM_QDOT) = J_rf_OB_c; // Jacobian of distance to contact point 6  
  J_phi.block(6,0, 1, NUM_QDOT) = J_rf_IB_c; // Jacobian of distance to contact point 7  
  J_phi.block(7,0, 1, NUM_QDOT) = J_rf_IF_c; // Jacobian of distance to contact point 8  

  // Left Foot
  phi[0] = lf_OF_pos[2]; // Distance to contact point 1
  phi[1] = lf_OB_pos[2]; // Distance to contact point 2
  phi[2] = lf_IB_pos[2]; // Distance to contact point 3
  phi[3] = lf_IB_pos[2]; // Distance to contact point 4  

  // Right Foot
  phi[4] = rf_OF_pos[2]; // Distance to contact point 5
  phi[5] = rf_OB_pos[2]; // Distance to contact point 6
  phi[6] = rf_IB_pos[2]; // Distance to contact point 7
  phi[7] = rf_IB_pos[2]; // Distance to contact point 8    

  // Left Foot
  sejong::Vector n1(3); n1[2] = 1.0; // Normal Vector for contact 1
  sejong::Vector n2 = n1; // Normal Vector for contact 2
  sejong::Vector n3 = n1; // Normal Vector for contact 3
  sejong::Vector n4 = n1; // Normal Vector for contact 4 

  // Right Foot
  sejong::Vector n5 = n1; // Normal Vector for contact 5
  sejong::Vector n6 = n1; // Normal Vector for contact 6
  sejong::Vector n7 = n1; // Normal Vector for contact 7 
  sejong::Vector n8 = n1; // Normal Vector for contact 8   

  sejong::Matrix J_c1 = J_lf_OF.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 1
  sejong::Matrix J_c2 = J_lf_OB.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 2  
  sejong::Matrix J_c3 = J_lf_IB.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 3
  sejong::Matrix J_c4 = J_lf_IF.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 4    

  sejong::Matrix J_c5 = J_rf_OF.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 5
  sejong::Matrix J_c6 = J_rf_OB.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 6  
  sejong::Matrix J_c7 = J_rf_IB.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 7
  sejong::Matrix J_c8 = J_rf_IF.block(3, 0, 3, NUM_QDOT); // Jacobian (x,y,z) at contact point 8    

  // Set Projected Normal Forces Direction
  sejong::Matrix N(NUM_QDOT, p);
  N.block(0,0, NUM_QDOT, 1) = J_c1.transpose()*n1;
  N.block(0,1, NUM_QDOT, 1) = J_c2.transpose()*n2; 
  N.block(0,2, NUM_QDOT, 1) = J_c3.transpose()*n3;
  N.block(0,3, NUM_QDOT, 1) = J_c4.transpose()*n4;

  N.block(0,4, NUM_QDOT, 1) = J_c5.transpose()*n5;
  N.block(0,5, NUM_QDOT, 1) = J_c6.transpose()*n6; 
  N.block(0,6, NUM_QDOT, 1) = J_c7.transpose()*n7;
  N.block(0,7, NUM_QDOT, 1) = J_c8.transpose()*n8;    

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

  sejong::Matrix D5 = D1; // Basis for contact 5
  sejong::Matrix D6 = D1; // Basis for contact 6
  sejong::Matrix D7 = D1; // Basis for contact 7    
  sejong::Matrix D8 = D1; // Basis for contact 8    


  // Set Projected Tangential Forces Direction
  sejong::Matrix B(NUM_QDOT, p*d); 
  B.block(0,0, NUM_QDOT, d) = J_c1.transpose()*D1;
  B.block(0,d, NUM_QDOT, d) = J_c2.transpose()*D2;  
  B.block(0,2*d, NUM_QDOT, d) = J_c3.transpose()*D3;
  B.block(0,3*d, NUM_QDOT, d) = J_c4.transpose()*D4;    

  B.block(0,4*d, NUM_QDOT, d) = J_c5.transpose()*D5;
  B.block(0,5*d, NUM_QDOT, d) = J_c6.transpose()*D6;  
  B.block(0,6*d, NUM_QDOT, d) = J_c7.transpose()*D7;
  B.block(0,7*d, NUM_QDOT, d) = J_c8.transpose()*D8;    

  // Unit e vecs and E binary matrix
  sejong::Vector e(d); // same size as number of friction basis direction 
  e.setOnes();
  // Diagonal e for each contact point
  sejong::Matrix E(p*d, p); 
  E.setZero();
  E.block(0, 0, d, 1) = e; 
  E.block(d, 1, d, 1) = e;
  E.block(2*d, 2, d, 1) = e;  
  E.block(3*d, 3, d, 1) = e;    

  E.block(4*d, 4, d, 1) = e;  
  E.block(5*d, 5, d, 1) = e;    
  E.block(6*d, 6, d, 1) = e;  
  E.block(7*d, 7, d, 1) = e;    

 
  // mu Matrix
  sejong::Matrix Mu = mu_static*sejong::Matrix::Identity(p, p);

  // Prepare the LCP problem
  double h = m_sim_rate; // timestep
  sejong::Matrix A_inv = Ainv_;  


  // Prepare Alpha Mu
  // 1st Row
  sejong::Matrix alpha_mu(p + p*d + p, p + p*d + p);
  alpha_mu.setZero();
  alpha_mu.block(0, 0, p, p) =  h*J_phi*A_inv*N;
  alpha_mu.block(0, p, p, p*d) =  h*J_phi*A_inv*B;

  // 2nd Row
  alpha_mu.block(p, 0,     p*d, p) = h*B.transpose()*A_inv*N;
  alpha_mu.block(p, p,     p*d, p*d) = h*B.transpose()*A_inv*B;
  alpha_mu.block(p, p+p*d, p*d, p) = E;

  // 3rd Row (Scaling for stabilization according to Tan, Jie. et al "Contact Handling for Articulated Rigid Bodies using LCP")
  alpha_mu.block(p+p*d, 0, p, p) = Mu * h; // Scaling by h for stabilizing contact constraints
  alpha_mu.block(p+p*d, p, p, p*d) = -E.transpose() * h;  // Scaling by h for stabilizing contact constraints

  

  // Prepare Beta
  sejong::Vector tau_star = A_*m_qdot + h*(m_tau - cori_ - grav_);
  sejong::Vector beta_mu(p + p*d + p);
  beta_mu.setZero();

  // 1st Row
  beta_mu.block(0,0, p, 1) = phi/h + J_phi*A_inv*tau_star;
  // 2nd Row
  beta_mu.block(p,0, p*d, 1) = B.transpose()*A_inv*tau_star;  

  
  sejong::Vector fn_fd_lambda(p + p*d + p);
  fn_fd_lambda.setZero();

  MobyLCPSolver l_mu;  
  bool result_mu = l_mu.lcp_lemke_regularized(alpha_mu, beta_mu, &fn_fd_lambda);

  sejong::Vector fn = fn_fd_lambda.block(0, 0, p, 1);
  sejong::Vector fd = fn_fd_lambda.block(p, 0, p*d, 1);

  N_mat = N;
  B_mat = B;
  fn_out = fn;
  fd_out = fd;  

//  bool result_mu = l_mu.lcp_fast(alpha_mu, beta_mu, &fn_fd_lambda);

}

double bound_torque(double torque_in){
	double torque_max = 100.0;
	if (torque_in >= torque_max){
		return torque_max;
	}else if (torque_in <= -torque_max){
		return -torque_max;
	}else{
		return torque_in;
	}

}

void LCP_Dyn_Simulator::MakeOneStepUpdate(){
	// Update Model
	UpdateModel();
	robot_model_->getInverseMassInertia(Ainv_);

	sejong::Matrix N_mat;
	sejong::Matrix B_mat;
	sejong::Vector fn_out;
	sejong::Vector fd_out;
	CreateFootContactModel(N_mat, B_mat, fn_out, fd_out);

	sejong::pretty_print(fn_out, std::cout, "Normal Force");

	// Get Torque Command
//	m_tau[3] = 10.0;
	for (size_t i = 0; i < NUM_ACT_JOINT; i++){
		m_tau[i + NUM_VIRTUAL] = 100.0*(0.0 - m_q[i + NUM_VIRTUAL]) + 10.0*(-m_qdot[i+NUM_VIRTUAL]);
		m_tau[i + NUM_VIRTUAL] = bound_torque(m_tau[i + NUM_VIRTUAL]);
	}

	// Fix xyz in the air
/*	m_tau[0] = 200. * (0.0 - m_q[0]) + 20.*(-m_qdot[0]);
	m_tau[1] = 200. * (0.0 - m_q[1]) + 20.*(-m_qdot[1]);
  	m_tau[2] = 2000. * (1.131 - m_q[2]) + 20.*(-m_qdot[2]);
*/

	// Perform Time Integratation ---------------------------	
	double dt = m_sim_rate;
	sejong::Vector qddot_next = Ainv_*(m_tau - cori_ - grav_ + N_mat*fn_out + B_mat*fd_out);
	//sejong::Vector qddot_next = Ainv_*(m_tau - cori_ - grav_);	
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