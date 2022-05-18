/*
 * ManualModeLogic.cpp
 *
 *  Created on: Apr 16, 2021
 *      Author: timityjoe
 */

#include "ManualModeLogic.h"
#include "../callbacks.h"
#include "../stm_utils.h"

#define DEBUG_MANUALMODE_CALC_YAWRATE_CMD 0
#define DEBUG_MANUALMODE_CTT 0

namespace atc_stm {


ManualModeLogic::ManualModeLogic()
{

}

ManualModeLogic::~ManualModeLogic() {

}

//------------------------------------------
void ManualModeLogic::doClickToTurn(ros::Publisher& cmd_vel_pub)
{
	if(atc_stm::firstCTTLoop)
	{
		publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Click To Turn state..");
		atc_stm::firstCTTLoop = false;
	}

	const double P_yaw = 0.1;
	const double MAX_YAW_RATE_RADSEC = 0.5;
	bool hasReached = false;
	const double cmd_yaw_rate_radsec = calculateYawRateCommand(P_yaw, MAX_YAW_RATE_RADSEC, hasReached);

	geometry_msgs::Twist cmd_velocity_msg;
//	cmd_velocity_msg.angular.z = -1.0*cmd_yaw_rate_radsec;
	cmd_velocity_msg.angular.z = cmd_yaw_rate_radsec;

#if DEBUG_MANUALMODE_CTT
	ROS_INFO("MANUAL_STEER 	CmdHeading:%.2f	getCurrHeading:%.2f	cmd_yaw_rate_radsec:%.2f \n",
			stm.getCmdHeading(),
			stm.getCurrHeading(),
			cmd_yaw_rate_radsec);
#endif
	cmd_vel_pub.publish(cmd_velocity_msg);

	if(hasReached)
	{
		atc_stm::movement_mode = atc_stm::MOVEMENT_MODE::MANUAL_STEER;
		atc_stm::agv_state = atc_stm::AGV_STATE::JOYSTICK_STEER;
		// Publish back to Rviz
		publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Click To Turn finished, going to Joystick Mode..");
	}
}

//------------------------------------------
double ManualModeLogic::calculateYawRateCommand(const double& P_yaw, const double& MAX_YAW_RATE_RADSEC, bool& hasReached)
{
//	double yaw_error_degs = cmd_heading_deg - curr_heading_degs;
//	double yaw_error_degs = cmd_odom_heading_deg - curr_odom_heading_degs;
	const double yaw_error_rad = atc_utils::shortest_angular_distance(atc_utils::from_degrees(curr_amclpose_heading_degs), atc_utils::from_degrees(cmd_odom_heading_deg));
	const double yaw_error_degs = atc_utils::to_degrees(yaw_error_rad);

#if DEBUG_MANUALMODE_CALC_YAWRATE_CMD
	ROS_INFO("StateTransitionManager::calculateYawRateCommand() cmd_heading_deg:%.2f curr_heading_degs:%.2f, yaw_error_degs:%.2f", cmd_heading_deg, curr_heading_degs, yaw_error_degs);
#endif
	// Simple P controller
	// Assume err = 5degs, we wish the
	// turn rate to be 0.25radsec
	// So P = 0.05;
	// turn rate to be 0.5radsec
	// So P = 0.1;
	double cmd_yaw_rate_radsec = 0;
	if(fabs(yaw_error_degs) > 5.0)
	{
		cmd_yaw_rate_radsec = P_yaw * yaw_error_degs;
		hasReached = false;
	}
	else
	{
		cmd_yaw_rate_radsec = 0;
		hasReached = true;
	}

#if DEBUG_MANUALMODE_CALC_YAWRATE_CMD
	ROS_INFO("	cmd_yaw_rate_radsec:%.2f", cmd_yaw_rate_radsec);
#endif

	// Limit yaw rate
	if(cmd_yaw_rate_radsec > MAX_YAW_RATE_RADSEC)
	{
		cmd_yaw_rate_radsec = MAX_YAW_RATE_RADSEC;
	}
	else if(cmd_yaw_rate_radsec < -MAX_YAW_RATE_RADSEC)
	{
		cmd_yaw_rate_radsec = -MAX_YAW_RATE_RADSEC;
	}

	return cmd_yaw_rate_radsec;
}


} /* namespace atc_stm */
