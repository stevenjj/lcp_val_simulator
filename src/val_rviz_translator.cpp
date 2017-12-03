#include <lcp_val_simulator/val_rviz_translator.hpp>

// Constructor
Val_Rviz_Translator::Val_Rviz_Translator(){}

// Destructor
Val_Rviz_Translator::~Val_Rviz_Translator(){}

void Val_Rviz_Translator::populate_joint_state_msg(const sejong::Vector & q, 
                                                    sensor_msgs::JointState & joint_state_msg){
  sensor_msgs::JointState joint_states;

  std::string joint_name;
  double joint_value = 0.0;  

  for (size_t i = 0; i < (NUM_QDOT-NUM_VIRTUAL); i++){
    joint_name = SJ_joint_index_to_name[i];
    joint_value = q[i + NUM_VIRTUAL];

    joint_states.name.push_back(joint_name);
    joint_states.position.push_back(joint_value);

    std::cout << joint_name << " " << joint_value << std::endl;
  }

  joint_state_msg = joint_states;
}

