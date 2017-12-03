#ifndef VAL_RVIZ_TRANSLATOR_H
#define VAL_RVIZ_TRANSLATOR_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Utils/wrap_eigen.hpp>
#include "valkyrie_definition.h"


class Val_Rviz_Translator{
public:
  std::map<int, std::string> SJ_joint_index_to_name = {
												          {0, "leftHipYaw"},
												          {1, "leftHipRoll"},
												          {2, "leftHipPitch"},
												          {3, "leftKneePitch"} ,
												          {4, "leftAnklePitch"},
												          {5, "leftAnkleRoll"},
												          {6, "rightHipYaw"},
												          {7, "rightHipRoll"},
												          {8, "rightHipPitch"},
												          {9, "rightKneePitch"},
												          {10, "rightAnklePitch"},
												          {11, "rightAnkleRoll"},                        
												          {12, "torsoYaw"},
												          {13, "torsoPitch"},
												          {14, "torsoRoll"},
												          {15, "leftShoulderPitch"},
												          {16, "leftShoulderRoll"},
												          {17, "leftShoulderYaw"},
												          {18, "leftForearmYaw"},
												          {19, "leftElbowPitch"},
												          {20, "lowerNeckPitch"},
												          {21, "neckYaw"},
												          {22, "upperNeckPitch"},
												          {23, "rightShoulderPitch"},
												          {24, "rightShoulderRoll"},
												          {25, "rightShoulderYaw"},
												          {26, "rightElbowPitch"},
												          {27, "rightForearmYaw"}
												       };

  void populate_joint_state_msg(const sejong::Vector & q,
                                 tf::Transform & world_to_pelvis_transform,
                                 sensor_msgs::JointState & joint_state_msg);

  Val_Rviz_Translator();
  ~Val_Rviz_Translator();  

};

namespace ValJoints{
	enum {
		hokuyo_joint = 0,
		torsoYaw,
		torsoPitch,
		torsoRoll,
		lowerNeckPitch,
		neckYaw,
		upperNeckPitch,
		rightShoulderPitch,
		rightShoulderRoll,
		rightShoulderYaw,
		rightElbowPitch,
		rightForearmYaw,
		rightWristRoll,
		rightWristPitch,
		rightThumbRoll,
		rightThumbPitch1,
		rightThumbPitch2,
		rightThumbPitch3,
		rightIndexFingerPitch1,
		rightIndexFingerPitch2,
		rightIndexFingerPitch3,
		rightMiddleFingerPitch1,
		rightMiddleFingerPitch2,
		rightMiddleFingerPitch3,
		rightPinkyPitch1,
		rightPinkyPitch2,
		rightPinkyPitch3,
		leftShoulderPitch,
		leftShoulderRoll,
		leftShoulderYaw,
		leftElbowPitch,
		leftForearmYaw,
		leftWristRoll,
		leftWristPitch,
		leftThumbRoll,
		leftThumbPitch1,
		leftThumbPitch2,
		leftThumbPitch3,
		leftIndexFingerPitch1,
		leftIndexFingerPitch2,
		leftIndexFingerPitch3,
		leftMiddleFingerPitch1,
		leftMiddleFingerPitch2,
		leftMiddleFingerPitch3,
		leftPinkyPitch1,
		leftPinkyPitch2,
		leftPinkyPitch3,
		rightHipYaw,
		rightHipRoll,
		rightHipPitch,
		rightKneePitch,
		rightAnklePitch,
		rightAnkleRoll,
		leftHipYaw,
		leftHipRoll,
		leftHipPitch,
		leftKneePitch,
		leftAnklePitch,
		leftAnkleRoll
	};
}

#endif