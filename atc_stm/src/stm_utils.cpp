/*
 * astmutils.cpp
 *
 *  Created on: Mar 10, 2021
 *      Author: timityjoe
 */

#include "stm_utils.h"
#include "atc_utils/angles.h"
#include "atc_utils/atc_utils.h"

#include <tf/transform_datatypes.h>

#include "callbacks.h"

#define DEBUG_ASTM_UTILS_ODOM 0

namespace atc_stm
{

geometry_msgs::Twist drive_panel_cmd_vel;

//----------------------------------------------------------------------------------------
void publishAgvStatus(const MOVEMENT_MODE& mode, const AGV_STATE& state, const std::string& status_string)
{
	publishAgvStatus(mode, state, status_string.c_str());
}

//----------------------------------------------------------------------------------------
void publishAgvStatus(const MOVEMENT_MODE& mode, const AGV_STATE& state, const char* status_char)
{
	atc_msgs::AGVStatus agv_status;
	agv_status.movement_mode = mode;
	agv_status.agv_state = state;
	agv_status.status_message = status_char;
	atc_stm::agv_status_pub.publish(agv_status);
}

//----------------------------------------------------------------------------------------
bool updateMovementMode(atc_msgs::Update_Movement_Mode::Request  &Req, atc_msgs::Update_Movement_Mode::Response  &Res)
{
	mtx.lock();
	ROS_WARN("1) ASTM - updateMovementMode()...");
	char cbuffer[200];
	std::string status_string;

	atc_stm::movement_mode = (atc_stm::MOVEMENT_MODE)(Req.movement_mode);

	// Set the AGV state, for the states that we know of
	atc_stm::agv_state = atc_stm::AGV_STATE::UNKNOWN_STATE;

	if(atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::MANUAL_STEER)
	{
		atc_stm::agv_state = atc_stm::AGV_STATE::JOYSTICK_STEER;
	}
	else if(atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::STOP)
	{
		atc_stm::agv_state = atc_stm::AGV_STATE::STOPPED;
	}
	else if(atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::WAYPOINT)
	{
		atc_stm::agv_state = atc_stm::AGV_STATE::PATROL;
	}


	Res.movement_mode = atc_stm::movement_mode;
	Res.agv_state = atc_stm::agv_state;

	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);

	sprintf(cbuffer, "Movement Mode update ok:%s,%s", movement_mode_string.c_str(), agv_state_string.c_str());
	status_string.append(cbuffer);
	Res.status_message = status_string;

	mtx.unlock();
	return true;
}

//----------------------------------------------------------------------------------------
void drivePanelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
  atc_stm::mtx.lock();
  //ROS_WARN("drivePanelCallback()");

  //firstCTTLoop = true;
  char cbuffer[200];
  std::string status_string;


  //atc_stm::stm_ptr->updateGoal(Req.x, Req.y, Req.theta);

//  atc_stm::MOVEMENT_MODE m_mode = atc_stm::MOVEMENT_MODE::MANUAL_STEER;
//  atc_stm::stm_ptr->updateMovementMode (m_mode);
//  atc_stm::AGV_STATE a_state = atc_stm::AGV_STATE::JOYSTICK_STEER;
//  atc_stm::stm_ptr->updateAgvState(a_state);

  drive_panel_cmd_vel = *cmd_vel_msg;

  //ROS_WARN("%s", status_string.c_str());

  atc_stm::mtx.unlock();
}

//------------------------------------------
void getCmdGoal(double& cmd_east_m,  double& cmd_north_m, double& cmd_hdg_deg)
{
	cmd_east_m = cmd_east_metres;
	cmd_north_m = cmd_north_metres;
	cmd_hdg_deg = cmd_odom_heading_deg;

	cmd_hdg_deg = atc_utils::constrainAngle_180(cmd_hdg_deg);

#if 1
		ROS_INFO("	(2-getCmdGoal()) cmd_x_m:%.2f, cmd_y_m:%.2f, cmd_odom_heading_deg:%.2f", cmd_east_metres, cmd_north_metres, cmd_odom_heading_deg);
#endif
}



//--------------------------------------------------------------------------------
//bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& goalReached)
//{
//	MoveBaseClient ac("move_base", true);
//    ac.cancelAllGoals();
//
//    move_base_msgs::MoveBaseGoal goal;
//    while (!ac.waitForServer(ros::Duration(5.0)))
//    {
//        ROS_INFO("Waiting for the move_base action server to come up...");
//    }
//
//#if DEBUG_SEND_GOAL
//	ROS_INFO("	sendGoal() - [move_base] action server up... ");
//#endif
//
//	//goal.target_pose.header.frame_id = "map";
//	goal.target_pose.header.frame_id = nav_goal_pose.header.frame_id;
//	goal.target_pose.header.stamp = ros::Time::now();
//	goal.target_pose = nav_goal_pose;
//
//	//---------------------------------------
//	// Debug
//	double yawRad = tf::getYaw(goal.target_pose.pose.orientation);
//
//	ROS_INFO(" 	(3-sendGoal())goal pose(x= %.2f y=%.2f yaw:%.2f) frame_id:%s",
//			goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
//			atc_utils::to_degrees(yawRad), goal.target_pose.header.frame_id.c_str());
//	//---------------------------------------
//
//	ac.sendGoal(goal);
//	ac.waitForResult();
//	std::string flag_success = "x";
//
//	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//	{
//		ROS_INFO("	Goal success");
//		flag_success = "true";
//		//msg_debug.data = req.wp_name +","+"success";
//
//		goalReached = true;
//		return true;
//	}
//	else
//	{
//		ROS_INFO("	Failed to achieved goal:%s ",  ac.getState().toString().c_str() );
//		flag_success = "false";
//		//msg_debug.data = req.wp_name +","+"fail";
//		return false;
//	}
//
//	ROS_INFO("	sendGoal() - UNKNOWN");
//	return false;
//
//}


} /* namespace astm */
