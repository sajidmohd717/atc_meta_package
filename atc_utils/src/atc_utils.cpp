/*
 * atc_utils.cpp
 *
 *  Created on: Mar 12, 2021
 *      Author: timityjoe
 */

#include "atc_utils/atc_utils.h"
#include "atc_utils/angles.h"

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define DEBUG_ATC_UTILS_SVC_CLIENT 1


namespace atc_utils
{

//------------------------------------------------------------------------
void imuMessageToRPY(const sensor_msgs::Imu::ConstPtr& imu_msg, double& rollRad, double& pitchRad, double& yawRad)
{
	tf::Quaternion quat_tf;
	tf::quaternionMsgToTF(imu_msg->orientation , quat_tf);

//	tf::Matrix3x3 m(quat_tf);
//	m.getRPY(rollRad, pitchRad, yawRad);
	quaternion2rpy(quat_tf, rollRad, pitchRad, yawRad);

}

//------------------------------------------------------------------------
void quaternionMsgToTF_no_warn(const geometry_msgs::Quaternion& msg, tf::Quaternion& bt)
{
	bt = tf::Quaternion(msg.x, msg.y, msg.z, msg.w);
//	if (fabs(bt.length2() - 1 ) > QUATERNION_TOLERANCE)
//	{
//	   ROS_WARN("MSG to TF: Quaternion Not Properly Normalized");
	   bt.normalize();
//	}
}

//------------------------------------------------------------------------
void quaternion2rpy(const tf::Quaternion& quat_tf, double& rollRad, double& pitchRad, double& yawRad)
{
	tf::Matrix3x3 m(quat_tf);
	m.getRPY(rollRad, pitchRad, yawRad);
}

//------------------------------------------------------------------------
void rpy2quaternion(const double& rollRad, const double& pitchRad, const double& yawRad, tf::Quaternion& quat_tf)
{
	quat_tf.setRPY(rollRad, pitchRad, yawRad);

	quat_tf = quat_tf.normalize();
}

//--------------------------------------------------------------------------------------------
void printMovementMode(const int m_mode)
{
	switch(m_mode) {
	    case 0 : std::cout << "UNKNOWN_MODE" << std::endl;
	             break;
	    case 1 : std::cout << "WAYPOINT" << std::endl;
	             break;
	    case 2 : std::cout << "MANUAL_STEER" << std::endl;
	             break;
	    case 3 : std::cout << "STOP" << std::endl;
	             break;
	}
}

//--------------------------------------------------------------------------------------------
std::string getMovementModeString(const int m_mode)
{
	std::string value;
	switch(m_mode)
	{
	    case 0 : value.append("UNKNOWN_MODE");
	             break;
	    case 1 : value.append("WAYPOINT");
	             break;
	    case 2 : value.append("MANUAL_STEER");
	             break;
	    case 3 : value.append("STOP");
	             break;
	}

	return value;
}

//--------------------------------------------------------------------------------------------
void printAgvState(const int a_state)
{
	switch(a_state) {
	    case 0 : std::cout << "UNKNOWN_STATE" << std::endl;
	             break;
	    case 1 : std::cout << "PATROL" << std::endl;
	             break;
	    case 2 : std::cout << "COARSE_GUIDANCE" << std::endl;
	             break;
	    case 3 : std::cout << "TERMINAL_GUIDANCE" << std::endl;
	             break;
	    case 4 : std::cout << "CTT_HEADING_HOLD" << std::endl;
	             break;
	    case 5 : std::cout << "JOYSTICK_STEER" << std::endl;
	             break;
	    case 6 : std::cout << "STOPPED" << std::endl;
	             break;
	    case 7 : std::cout << "E_STOPPED (please unlatch to continue)" << std::endl;
	             break;
	}
}

//--------------------------------------------------------------------------------------------
std::string getAgvStateString(const int a_state)
{
	std::string value;
	switch(a_state)
	{
	    case 0 : value.append("UNKNOWN_STATE");
	             break;
	    case 1 : value.append("PATROL");
	             break;
	    case 2 : value.append("COARSE_GUIDANCE");
	             break;
	    case 3 : value.append("TERMINAL_GUIDANCE");
	             break;
	    case 4 : value.append("CTT_HEADING_HOLD");
	             break;
	    case 5 : value.append("JOYSTICK_STEER");
	             break;
	    case 6 : value.append("STOPPED");
	             break;
	    case 7 : value.append("E_STOPPED (please unlatch to continue)");
	             break;
	}

	return value;
}

//--------------------------------------------------------------------------------------------
bool debugServiceCllient(ros::ServiceClient& svc_client)
{
#if DEBUG_ATC_UTILS_SVC_CLIENT
	ROS_INFO("debugServiceCllient() ");
#endif

	if(!svc_client.isValid())
	{
		ROS_INFO("debugServiceCllient() - !isValid(), init the service client first..?");
		return false;
	}
	if(!svc_client.exists())
	{
		ROS_INFO("debugServiceCllient() -  !exists(), service server application not running..?");
		return false;
	}

	ROS_INFO("debugServiceCllient() ok");
	return true;
}

//--------------------------------------------------------------------------------------------
float limitValuef(const float& valueIn, const float& MAX_ABS_VALUE)
{
	// Limit yaw rate
	float valueOut = valueIn;
	if(valueIn > MAX_ABS_VALUE)
	{
		valueOut = MAX_ABS_VALUE;
	}
	else if(valueIn < -MAX_ABS_VALUE)
	{
		valueOut = -MAX_ABS_VALUE;
	}

	return valueOut;
}

} /* namespace atc_utils */
