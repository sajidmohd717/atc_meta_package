/*
 * atc_utils.h
 *
 *  Created on: Mar 12, 2021
 *      Author: timityjoe
 */

#ifndef SRC_ATC_UTILS_H_
#define SRC_ATC_UTILS_H_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>

namespace atc_utils
{
	void imuMessageToRPY(const sensor_msgs::Imu::ConstPtr& imu_msg, double& rollRad, double& pitchRad, double& yawRad);

	void quaternionMsgToTF_no_warn(const geometry_msgs::Quaternion& msg, tf::Quaternion& bt);
	void quaternion2rpy(const tf::Quaternion& quat_tf, double& rollRad, double& pitchRad, double& yawRad);
	void rpy2quaternion(const double& rollRad, const double& pitchRad, const double& yawRad, tf::Quaternion& quat_tf);

	void printMovementMode(const int m_mode);
	std::string getMovementModeString(const int m_mode);

	void printAgvState(const int a_state);
	std::string getAgvStateString(const int a_state);

	bool debugServiceCllient(ros::ServiceClient& svc_client);

	float limitValuef(const float& valueIn, const float& MAX_ABS_VALUE);

} /* namespace atc_utils */

#endif /* SRC_ATC_UTILS_H_ */
