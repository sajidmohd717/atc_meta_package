/*
 * StateTransitionManager.cpp
 *
 *  Created on: Mar 3, 2021
 *      Author: timityjoe
 */

#include "StateTransitionManager.h"
#include "tf/LinearMath/Vector3.h"
#include "atc_utils/angles.h"
#include "atc_utils/Vector2.hpp"

#include "callbacks.h"
#include "stm_utils.h"

#define DEBUG_STM 0
#define DEBUG_STM_CALC_YAWRATE_CMD 0
#define DEBUG_STM_CALC_APPROX_GOAL 0

#define DEBUG_STM_WAYPOINTMODES_CORE 1
#define DEBUG_STM_WAYPOINTMODES_ADDITIONAL 0


namespace atc_stm {

//------------------------------------------
StateTransitionManager::StateTransitionManager(std::mutex* mtxPtr, AprilTagLogic* aprilTagLogicPtr, WaypointLogic* 	waypointLogicPtr):
		aprilTagLogic_ptr(aprilTagLogicPtr),
		waypointLogic_ptr(waypointLogicPtr),
		detectorLogic(waypointLogicPtr)
{

}

//------------------------------------------
StateTransitionManager::~StateTransitionManager()
{

}

//------------------------------------------
void StateTransitionManager::setNodeHandle(ros::NodeHandle& nh)
{
	nh_ptr = &nh;
//	specific_wp_client = nh_ptr->serviceClient<atc_msgs::Run_Specific_Wp>("waypoint_server/run_specific_wp");
}

//------------------------------------------
//void StateTransitionManager::reset()
//{
//#if 0
//	ROS_INFO("StateTransitionManager::reset()");
//#endif
//
//	aprilTagLogic_ptr->reset();
//	detectorLogic.reset();
//}

//------------------------------------------
void StateTransitionManager::updateLinearSpeed(const double& speed_ms)
{
	curr_linear_speed_ms = speed_ms;
}
//------------------------------------------
void StateTransitionManager::updateAmclPose(const double& x_m, const double& y_m, const double& odom_hdg_deg)
{
	curr_east_metres = x_m;
	curr_north_metres = y_m;
	curr_amclpose_heading_degs = odom_hdg_deg;
}


//------------------------------------------
void StateTransitionManager::updateHeadingAndYawRate (const double& hdg_degs, const double& yaw_rate_radsec)
{
	curr_heading_degs = hdg_degs;
	curr_yaw_rate_radsec = yaw_rate_radsec;

#if 0
	ROS_INFO("StateTransitionManager::updateHeadingAndYawRate() curr_heading_degs:%.2f, curr_yaw_rate_radsec:%.2f ", curr_heading_degs, curr_yaw_rate_radsec);
#endif
}

//------------------------------------------
void StateTransitionManager::updateCommandedOdomHeading (const double& hdg_degs)
{
	cmd_odom_heading_deg = hdg_degs;
}

//------------------------------------------
bool StateTransitionManager::doWaypointDetectorMode()
{

#if DEBUG_STM_WAYPOINTMODES_CORE
		ROS_INFO("StateTransitionManager::doWaypointDetectorMode() bNavToTrolley:%i ", bNavToTrolley);
#endif

		// Trolley detector mode
		if(atc_stm::bNavToTrolley)
		{
			if(detectorLogic.bDetectorHasTarget && !detectorLogic.bApproxGoalReached)
			{
#if DEBUG_STM_WAYPOINTMODES_CORE
				ROS_INFO("	STM-WPT-Det 1: Heading to approx goal... bDetectorGoalSent:%i", atc_stm::bDetectorGoalSent);
#endif
				geometry_msgs::PoseStamped approx_goal_pose;
				detectorLogic.calculateApproximateGoal();
				detectorLogic.doNavToTrolley(approx_goal_pose);

				if(!atc_stm::bDetectorGoalSent)
				{
					atc_stm::stopGoals();
					atc_stm::sendGoal(approx_goal_pose, detectorLogic.bApproxGoalReached);
					atc_stm::bDetectorGoalSent = true;
				}

#if DEBUG_STM_WAYPOINTMODES_CORE
				ROS_INFO("	STM-WPT-Det 1: Stop & sending... bDetectorGoalSent:%i", atc_stm::bDetectorGoalSent);
#endif

				return true;
			}
			else if(detectorLogic.bDetectorHasTarget && detectorLogic.bApproxGoalReached)
			{
#if DEBUG_STM_WAYPOINTMODES_CORE
				ROS_INFO("	STM-WPT-Det 2: Approx goal reached...bDetectorGoalSent:%i", atc_stm::bDetectorGoalSent);
#endif
				// Reset the triggers
				detectorLogic.bDetectorHasTarget = false;
				detectorLogic.bApproxGoalReached = false;
				atc_stm::bDetectorGoalSent = false;
				atc_stm::bNavToTrolley = false;

				atc_stm::movement_mode = atc_stm::MOVEMENT_MODE::MANUAL_STEER;
				atc_stm::agv_state = atc_stm::AGV_STATE::JOYSTICK_STEER;
				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Approx goal reached, going to Joystick Mode..");

				return true;
			}
			else
			{
#if DEBUG_STM_WAYPOINTMODES_CORE
				ROS_WARN("	STM-WPT-Det 3: UNKNOWN STATE! (Default Waypoint Logic)");
#endif
				// Reset the triggers
				detectorLogic.bDetectorHasTarget = false;
				detectorLogic.bApproxGoalReached = false;
				atc_stm::bDetectorGoalSent = false;
				atc_stm::bNavToTrolley = false;

//				atc_stm::checkGoalStatus(detectorLogic.bGoalSent, detectorLogic.bApproxGoalReached);
				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "In Waypoint Patrol mode(3)..");
				return true;
			}

		}
		else
		{
#if DEBUG_STM_WAYPOINTMODES_ADDITIONAL
			ROS_WARN("	STM-WPT-Det 4: UNKNOWN STATE! (Default Waypoint Logic)");
#endif
//			atc_stm::checkGoalStatus(detectorLogic.bGoalSent, detectorLogic.bApproxGoalReached);
//			publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "In Waypoint Patrol mode(4)..");
			return true;
		}

#if DEBUG_STM_WAYPOINTMODES_CORE
		ROS_WARN("	STM-WPT-Det 5: UNKNOWN STATE! bNavToTrolley:%i, bDetectorHasTarget:%i, bApproxGoalReached%i",
				atc_stm::bNavToTrolley, detectorLogic.bDetectorHasTarget, detectorLogic.bApproxGoalReached);
#endif
//		atc_stm::checkGoalStatus(detectorLogic.bGoalSent, detectorLogic.bApproxGoalReached);
		publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "In Waypoint Patrol UNKNOWN STATE(5)..");

		return false;
}

//------------------------------------------
bool StateTransitionManager::doWaypointDockingMode(ros::Publisher& cmd_vel_pub)
{
	if(atc_stm::bDockToAprilTag)
	{

		if(!aprilTagLogic_ptr->stagingGoalReached)
		{
#if DEBUG_STM_WAYPOINTMODES_ADDITIONAL
		ROS_INFO("	STM-WPT-AT 1: Heading to staging goal...");
#endif

			atc_stm::agv_state = atc_stm::AGV_STATE::COARSE_GUIDANCE;
			publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "AprilTag detected, going to Staging Goal..");

			geometry_msgs::PoseStamped staging_goal_pose;
			if(!aprilTagLogic_ptr->calcMotionGoal(staging_goal_pose))
			{
				ROS_INFO("	STM-WPT-AT 1: calcMotionGoal() failed!");
				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "STM-WPT-AT 1: calcMotionGoal() failed!");
				return false;
			}

			if(aprilTagLogic_ptr->checkMotionGoalChanged())
			{
				ROS_INFO("	STM-WPT-AT 1: checkMotionGoalChanged() new goal, sending..");
				aprilTagLogic_ptr->sendMotionGoal();
			}

			return true;
		}
		else if(aprilTagLogic_ptr->stagingGoalReached && !aprilTagLogic_ptr->dockingGoalReached)
		{
#if DEBUG_STM_WAYPOINTMODES_ADDITIONAL
		ROS_INFO("	STM-WPT-AT 2: Steer for docking");
#endif
			geometry_msgs::PoseStamped staging_goal_pose;
			if(!aprilTagLogic_ptr->calcMotionGoal(staging_goal_pose))
			{
				ROS_INFO("	STM-WPT-AT 2: calcMotionGoal() failed!");
				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "STM-WPT-AT 1: calcMotionGoal() failed!");
				return false;
			}

			double linearSpdCmd;
			double yawRateCmd;
			if(!aprilTagLogic_ptr->calcDockingCmds(linearSpdCmd, yawRateCmd))
			{
				ROS_INFO("	STM-WPT-AT 2: calcDockingCmds() failed!");
				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "STM-WPT-AT 2: calcDockingCmds() failed!");
				return false;
			}

			if(aprilTagLogic_ptr->checkMotionGoalChanged())
			{
				ROS_INFO("	STM-WPT-AT 2: checkMotionGoalChanged() new goal, resetting stagingGoalReached...");
				aprilTagLogic_ptr->stagingGoalReached = false;
			}

			atc_stm::agv_state = atc_stm::AGV_STATE::TERMINAL_GUIDANCE;

			char cbuffer[200];
			sprintf(cbuffer, "Docking to Trolley, TagID:%i, SampleSize:%i", aprilTagLogic_ptr->getClosestTrolleyID(), aprilTagLogic_ptr->marker_pose_vec_.size());
			std::string status_string(cbuffer);
			publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, status_string);

			geometry_msgs::Twist cmd_velocity_msg;
			cmd_velocity_msg.angular.z = -1.0*yawRateCmd;
			cmd_velocity_msg.linear.x = linearSpdCmd;
			cmd_vel_pub.publish(cmd_velocity_msg);

			return true;
		}
		else if(aprilTagLogic_ptr->dockingGoalReached)
		{
#if DEBUG_STM_WAYPOINTMODES_CORE
			ROS_INFO("	STM-WPT-AT 3: Docked! stagingGoalReached:%i, dockingGoalReached:%i ",
					aprilTagLogic_ptr->stagingGoalReached, aprilTagLogic_ptr->dockingGoalReached);
#endif

			// Mod by Tim (29th July 2022)
			// Reset the states and put to manual mode
			aprilTagLogic_ptr->reset();

			atc_stm::movement_mode = atc_stm::MOVEMENT_MODE::MANUAL_STEER;
			atc_stm::agv_state = atc_stm::AGV_STATE::JOYSTICK_STEER;
			publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Trolley docked, going to Joystick Mode..");

			return true;

		}
		else
		{
#if DEBUG_STM_WAYPOINTMODES_CORE
			ROS_WARN("	STM-WPT-AT 4: UNKNOWN STATE! bDockToAprilTag:%i, stagingGoalReached:%i, dockingGoalReached:%i",
					bDockToAprilTag, aprilTagLogic_ptr->stagingGoalReached, aprilTagLogic_ptr->dockingGoalReached);
#endif
			return false;
		}

	}

#if DEBUG_STM_WAYPOINTMODES_CORE
	ROS_WARN("	STM-WPT-AT 5: UNKNOWN STATE! bDockToAprilTag:%i, stagingGoalReached:%i, dockingGoalReached:%i",
			bDockToAprilTag, aprilTagLogic_ptr->stagingGoalReached, aprilTagLogic_ptr->dockingGoalReached);
#endif

	return false;
}




} /* namespace astm */
