#include <lcp_val_simulator/val_rviz_translator.hpp>

// Constructor
Val_Rviz_Translator::Val_Rviz_Translator(){}

// Destructor
Val_Rviz_Translator::~Val_Rviz_Translator(){}

void Val_Rviz_Translator::populate_joint_state_msg(const sejong::Vector & q, 
                                                    tf::Transform & world_to_pelvis_transform,
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

  tf::Transform transform;

  transform.setOrigin( tf::Vector3(q[0], q[1], q[2]) ); //x, y, z
  tf::Quaternion quat_world_to_pelvis(q[3], q[4], q[5], q[NUM_Q-1]); //x, y, z, w
  transform.setRotation(quat_world_to_pelvis);

  // Set Output
  joint_state_msg = joint_states;
  world_to_pelvis_transform = transform;
}



    

