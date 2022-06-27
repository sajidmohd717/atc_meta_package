/*
 * callbacks.h
 *
 *  Created on: Apr 16, 2021
 *      Author: timityjoe
 */

#ifndef ATC_STM_SRC_CALLBACKS_H_
#define ATC_STM_SRC_CALLBACKS_H_

/* --Includes-- */

#include <mutex>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "atc_utils/angles.h"
#include "atc_utils/atc_utils.h"

// ROS Messages
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

// ATC Messages / services
#include "atc_msgs/Detector.h"
#include "atc_msgs/AGVStatus.h"
#include "atc_msgs/Click_To_Turn.h"
#include "atc_msgs/Detector_Has_Solution.h"
#include "atc_msgs/AprilTag_Has_Solution.h"
#include "atc_msgs/Navigate_To_Trolley.h"
#include "atc_msgs/Dock_To_Tag.h"
#include "atc_msgs/Stop_To_STM.h"
// *New additions (28th July 2021)
#include "atc_msgs/Reset_ClearCostMap.h"
//#include "atc_msgs/Charge_AGV.h"

// Main controller
#include "StateTransitionManager.h"

namespace atc_stm {

	class StateTransitionManager;

//------------------------------------------------------------------------
	// Variables
	extern std::mutex mtx;
	extern bool firstCTTLoop;
	extern StateTransitionManager* stm_ptr;
	extern ros::Publisher agv_status_pub;
	
	// For Trolley detection
	extern ros::Publisher agv_detector_pub;	// 1) Msg to notify rviz
	extern bool bNavToTrolley;				// 2) Service to cfm engagement with trolley
	extern bool bDetectorGoalSent;

	// For docking ops
	extern ros::Publisher agv_docking_pub;	// 1) Msg to notify rviz
	extern bool bDockToAprilTag;				// 2) Service to cfm docking with tag

//	extern ros::ServiceClient specific_wp_client;
//	extern atc_msgs::Run_Specific_Wp run_specific_wp_srv;

//    extern std::mutex mtx_stopAgv;
	extern bool bStopAGV;

	extern MOVEMENT_MODE movement_mode;
	extern AGV_STATE agv_state;

	extern double curr_heading_degs;
	extern double curr_amclpose_heading_degs;
	extern double curr_yaw_rate_radsec;
	extern double curr_east_metres;
	extern double curr_north_metres;
	extern double curr_linear_speed_ms;

	extern double cmd_east_metres;
	extern double cmd_north_metres;
	extern double cmd_odom_heading_deg;
//------------------------------------------------------------------------
// Functions
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
	
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
	
	void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_msg);

	void detectorCallback(const atc_msgs::Detector::ConstPtr& detector_msg);
	
	void stopToStmCallback(const atc_msgs::Stop_To_STM::ConstPtr& stopToStmMsg);

	bool clickToTurn(atc_msgs::Click_To_Turn::Request  &Req, atc_msgs::Click_To_Turn::Response  &Res);

	bool navigateToTrolley(atc_msgs::Navigate_To_Trolley::Request  &Req, atc_msgs::Navigate_To_Trolley::Response  &Res);

	bool dockToTag(atc_msgs::Dock_To_Tag::Request  &Req, atc_msgs::Dock_To_Tag::Response  &Res);

	bool stopGoals();
	bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& goalReached);
//	bool sendSteeringCmds();

	// *New additions (28th July 2021)
	bool resetClearCostmapCallback(atc_msgs::Reset_ClearCostMap::Request  &Req, atc_msgs::Reset_ClearCostMap::Response  &Res);
//	bool chargeAgvCallback(atc_msgs::Charge_AGV::Request  &Req, atc_msgs::Charge_AGV::Response  &Res);



} /* namespace atc_stm */

#endif /* ATC_STM_SRC_CALLBACKS_H_ */
