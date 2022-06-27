/*
 * DetectorLogic.cpp
 *
 *  Created on: Apr 16, 2021
 *      Author: timityjoe
 */

#include "DetectorLogic.h"
#include "atc_utils/Vector2.hpp"
#include "../callbacks.h"
#include "../stm_utils.h"

#define DEBUG_DETECTORLOGIC_STATES 1

namespace atc_stm {


DetectorLogic::DetectorLogic(WaypointLogic* waypointLogicPtr):
		waypointLogic_ptr(waypointLogicPtr),
		first_detector_Loop(true)
{
	box_width = 0.0;
	box_height = 0.0;
	box_pixelPosRight = 0.0;

	bDetectorHasTarget = false;
	bApproxGoalReached = false;
	atc_stm::bDetectorGoalSent = false;
	bNavToTrolley = false;
}

DetectorLogic::~DetectorLogic() {

}

//------------------------------------------
void DetectorLogic::reset()
{
	bDetectorHasTarget = false;
	bApproxGoalReached = false;
	atc_stm::bDetectorGoalSent = false;
	bNavToTrolley = false;
}


//------------------------------------------
void DetectorLogic::doNavToTrolley(geometry_msgs::PoseStamped& approx_goal_pose)
{
#if DEBUG_DETECTORLOGIC_STATES
	ROS_INFO("DetectorLogic::doNavToTrolley() bDetectorGoalSent:%i", atc_stm::bDetectorGoalSent);
#endif
	// Stop the waypoint logic (if any ongoing)
//	std_msgs::Int8 stop_msg;
//	stop_msg.data = 1;
//	waypointLogic_ptr->CallbackStop(stop_msg);
	atc_stm::agv_state = atc_stm::AGV_STATE::COARSE_GUIDANCE;
	publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Trolley detected! going to Staging Goal..");

	//geometry_msgs::PoseStamped approx_goal_pose;
	double cmd_east_m;
	double cmd_north_m;
	double cmd_hdg_deg;
	atc_stm::getCmdGoal(cmd_east_m, cmd_north_m, cmd_hdg_deg);

	approx_goal_pose.header.frame_id = "map";
	approx_goal_pose.header.stamp = ros::Time::now();
	approx_goal_pose.pose.position.x = cmd_east_m;
	approx_goal_pose.pose.position.y = cmd_north_m;
	double cmd_hdg_rad = atc_utils::from_degrees(cmd_hdg_deg);

	//-----------------------------------------------------------
	// Adapt; heading range is 0-360deg clockwise +ve
	//       ROS range is 180 to -180:
	//       - clockwise 180 is -ve		  (ie: heading 0 clockwise towards 180deg)
	//       - anti clockwise 180 is +ve  (ie: heading 360 anticlockwise towards 180deg)
//	cmd_hdg_rad = -1.0*atc_utils::normalize_angle(cmd_hdg_rad);
	//-----------------------------------------------------------

	approx_goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(cmd_hdg_rad);
//	atc_stm::sendGoal(approx_goal_pose, bApproxGoalReached);
//	bGoalSent = true;
}

//------------------------------------------
// Called in callbacks.h
// Assumes imu and odom callback have been called
// Returns true, if successive goal is detected to be changed...
//bool DetectorLogic::calculateApproximateGoal(const double& boxWidth, const double& boxHeight, const double& pixelPosRight)
bool DetectorLogic::calculateApproximateGoal()
{
	// Calculate Area
	const double boxArea = box_width*box_height;

	// Calculate range and heading
	const double tgtRangeMeters = approxTgtRange(boxArea) * 0.8;	// Set to 80%, if not goal will collide w the trolley...
	const double FOV_deg = 85.2;
	const double tgtHeadingDegs = approxTgtHeading(box_pixelPosRight, FOV_deg);

	double new_cmd_east_metres = 0.0;
	double new_cmd_north_metres = 0.0;
	tgtRangeHeadingToGoal(tgtRangeMeters, tgtHeadingDegs, new_cmd_east_metres, new_cmd_north_metres);
	// Cmd heading should factor in tgt
	//cmd_heading_deg = curr_heading_degs;
	cmd_odom_heading_deg = tgtHeadingDegs;

	if(first_detector_Loop)
	{
		cmd_east_metres = new_cmd_east_metres;
		cmd_north_metres = new_cmd_north_metres;
		first_detector_Loop = false;

#if DEBUG_DETECTOR_CALC_APPROX_GOAL
		ROS_INFO("StateTransitionManager::calculateApproximateGoal() 1st loop cmd_x_m:%.2f, cmd_y_m:%.2f, cmd_heading_deg:%.2f", cmd_east_metres, cmd_north_metres, cmd_heading_deg);
#endif
	}

#if 1
	ROS_INFO("StateTransitionManager::calculateApproximateGoal() boxArea:%.2f, tgtRangeMeters:%.2f, tgtHeadingDegs:%.2f ", boxArea, tgtRangeMeters, tgtHeadingDegs);
#endif

#if DEBUG_DETECTOR_CALC_APPROX_GOAL
	ROS_INFO("StateTransitionManager::calculateApproximateGoal() cmd_x_m:%.2f, cmd_y_m:%.2f, cmd_heading_deg:%.2f", cmd_east_metres, cmd_north_metres, cmd_heading_deg);
#endif

	if( fabs(new_cmd_east_metres - cmd_east_metres) > 1.0 ||
			  fabs(new_cmd_north_metres - cmd_north_metres) > 1.0 )
	{
#if 1
		ROS_INFO("	DetectorLogic::calculateApproximateGoal() Goal update...");
#endif
		cmd_east_metres = new_cmd_east_metres;
		cmd_north_metres = new_cmd_north_metres;
		atc_stm::bDetectorGoalSent = false;

		return true;
	}

#if 0
		ROS_INFO("	(1-calculateApproximateGoal()) cmd_x_m:%.2f, cmd_y_m:%.2f, cmd_heading_deg:%.2f", cmd_east_metres, cmd_north_metres, cmd_heading_deg);
#endif

	return false;
}

//------------------------------------------
// Emperical data from Gazebo dist vs tag area:
// Tag area was covariance of about 10%
// Dist(Y)  |   Area(X)
// 9m           25500
// 8.1m         36500
// 7.98m        41000
// 6.4m         50000
// 3.8m         200000
// 2.33m        600500
// From https://mycurvefit.com/
// y = 0.9867936 + (21.45595 - 0.9867936)/(1 + (x/13574.02)^0.6950235)
double DetectorLogic::approxTgtRange(const double& boxArea)
{
	double val1 = boxArea/13574.02;
	double val2 = pow (val1, 0.6950235);
	//double rangeMeters = 0.9867936 + (21.45595 - 0.9867936)/(1 + (boxArea/13574.02)^0.6950235);
	double rangeMeters = 0.9867936 + (21.45595 - 0.9867936)/(1 + val2);
	return rangeMeters;
}

//------------------------------------------
// Mod by Tim: Change from IMU call back to Odom, for the heading reference..
// Odom heading, left  half 0 to 180
//               right half 0 t0 -180
double DetectorLogic::approxTgtHeading(const double& pixelPosRight, const double& FOV_deg)
{
	const double boreSightError = 0.5 - pixelPosRight;
	const double yaw_error_degs = boreSightError * FOV_deg;
	const double tgtHeading = curr_amclpose_heading_degs + yaw_error_degs;

#if DEBUG_DETECTORLOGIC_STATES
	ROS_INFO("DetectorLogic::approxTgtHeading() curr_odom_heading_degs:%.2f, pixelPosRight:%.2f boreSightError:%.2f, yaw_error_degs:%.2f, tgtHeading:%.2f",
			atc_stm::curr_amclpose_heading_degs, pixelPosRight, boreSightError, yaw_error_degs, tgtHeading);
#endif

	return tgtHeading;
}

//------------------------------------------
// Given target range and heading, convert to goal for trolley to head towards
void DetectorLogic::tgtRangeHeadingToGoal(const double& tgtRangeMeters, const double& tgtHeadingDegs, double& cmd_east_m, double& cmd_north_m)
{
	// Angle is in radians
	atc_utils::Vector2 curr_pos_glob(curr_east_metres, curr_north_metres);
//	const double tgtHeadingRads = atc_utils::from_degrees(tgtHeadingDegs);
//	atc_utils::Vector2 offset_glob = atc_utils::Vector2::FromPolar(tgtRangeMeters, tgtHeadingRads);

	// Mod by Tim:
	const double tgtHeadingRads_360 = atc_utils::normalize_angle_positive(atc_utils::from_degrees(tgtHeadingDegs));
	atc_utils::Vector2 offset_glob = atc_utils::Vector2::FromPolar(tgtRangeMeters, tgtHeadingRads_360);

	atc_utils::Vector2 goal_glob = curr_pos_glob + offset_glob;

	cmd_east_m = goal_glob.X;
	cmd_north_m = goal_glob.Y;

#if 1
	ROS_INFO("StateTransitionManager::tgtRangeHeadingToGoal() tgtHdg:%.2f, d_x:%.2f, d_y:%.2f, cmd_east_m:%.2f, cmd_north_m:%.2f",
			tgtHeadingDegs, offset_glob.X, offset_glob.Y, cmd_east_m, cmd_north_m);
#endif

#if 0
	ROS_INFO("StateTransitionManager::tgtRangeHeadingToGoal() cmd_x_m:%.2f, cmd_y_m:%.2f \n", cmd_east_m, cmd_north_m);
#endif
}


} /* namespace atc_stm */


