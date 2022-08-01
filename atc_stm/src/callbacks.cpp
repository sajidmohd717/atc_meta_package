/*
 * callbacks.cpp
 *
 *  Created on: Apr 16, 2021
 *      Author: timityjoe
 */

#include "callbacks.h"
#include "stm_utils.h"

#define DEBUG_CALLBACKS_SENDGOAL 1

namespace atc_stm {

std::mutex mtx;
bool firstCTTLoop = true;
StateTransitionManager* stm_ptr = NULL;
ros::Publisher agv_status_pub;

// For Trolley detection
ros::Publisher agv_detector_pub;	// 1) Msg to notify rviz
bool bNavToTrolley = false;			// 2) Service to cfm engagement with trolley
bool bDetectorGoalSent = false;

// For docking ops
ros::Publisher agv_docking_pub;	// 1) Msg to notify rviz
bool bDockToAprilTag = false;				// 3) Service to cfm docking with tag

//ros::ServiceClient specific_wp_client;
//atc_msgs::Run_Specific_Wp run_specific_wp_srv;
//std::mutex mtx_stopAgv;
bool bStopAGV = false;

MOVEMENT_MODE movement_mode = MOVEMENT_MODE::UNKNOWN_MODE;
AGV_STATE agv_state = AGV_STATE::UNKNOWN_STATE;

double curr_heading_degs;
double curr_amclpose_heading_degs;
double curr_yaw_rate_radsec;
double curr_east_metres;
double curr_north_metres;
double curr_linear_speed_ms;

double cmd_east_metres;
double cmd_north_metres;
double cmd_odom_heading_deg;

//----------------------------------------------------------------------------------------
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  mtx.lock();
  //ROS_INFO("imuCallback()");

  double rollRad, pitchRad, yawRad;
  atc_utils::imuMessageToRPY(imu_msg, rollRad, pitchRad, yawRad);

  double headingDegs = atc_utils::to_degrees( atc_utils::normalize_angle_positive(-yawRad) );
//  double yawRateRadsec = -1.0*(imu_msg->angular_velocity.z);
  double yawRateRadsec = imu_msg->angular_velocity.z;

  stm_ptr->updateHeadingAndYawRate(headingDegs, yawRateRadsec);

  mtx.unlock();

}

//----------------------------------------------------------------------------------------
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  mtx.lock();
  stm_ptr->updateLinearSpeed(odom_msg->twist.twist.linear.x);
  mtx.unlock();
}

//----------------------------------------------------------------------------------------
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_msg)
{
	mtx.lock();

	//--------------------------------------------------
	// Mod by Tim: Change from IMU call back to Odom, for the heading reference..
	double rollRad, pitchRad, yawRad;
	tf::Quaternion quat_tf1;
	atc_utils::quaternionMsgToTF_no_warn(amcl_pose_msg->pose.pose.orientation, quat_tf1);
	atc_utils::quaternion2rpy(quat_tf1, rollRad, pitchRad, yawRad);

	const double odom_heading_degs = atc_utils::to_degrees( yawRad );
	//--------------------------------------------------
	stm_ptr->updateAmclPose(amcl_pose_msg->pose.pose.position.x,
				  amcl_pose_msg->pose.pose.position.y,
				  odom_heading_degs);

#if DEBUG_ASTM_UTILS_ODOM
	ROS_INFO("ASTM::odomCallback() x:%.2f, y:%.2f, spd:%.2f ",
			odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->twist.twist.linear.x);
#endif

	mtx.unlock();
}


//----------------------------------------------------------------------------------------
void detectorCallback(const atc_msgs::Detector::ConstPtr& detector_msg)
{
	//ROS_INFO("detectorCallback()");
	  mtx.lock();
	  //ROS_WARN("detectorCallback()...");

	  char cbuffer[200];
	  std::string status_string;

	  // Send out the notification to rviz (for pushbutton)
	  // Reciprocal (hasSolution = true) is in main()
	  atc_msgs::Detector_Has_Solution agv_detect_pushbutton;

	  if(detector_msg->boxes.size() > 0)
	  {
		  //ROS_INFO("detectorCallback() - %i Trolley(s) detected:", detector_msg->boxes.size());
		  agv_detect_pushbutton.hasSolution = true;
		  stm_ptr->detectorLogic.bDetectorHasTarget = true;
//		  bool goalChanged = stm_ptr->detectorLogic.calculateApproximateGoal(detector_msg->boxes[0].width, detector_msg->boxes[0].height, detector_msg->boxes[0].pixelPosRight);
		  stm_ptr->detectorLogic.box_width = detector_msg->boxes[0].width;
		  stm_ptr->detectorLogic.box_height = detector_msg->boxes[0].height;
		  stm_ptr->detectorLogic.box_pixelPosRight = detector_msg->boxes[0].pixelPosRight;

		  std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
		  std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);

//		  if(!goalChanged)
//		  {
//		      sprintf(cbuffer, "Trolley detected:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
//		  }
//		  else
//		  {
//			  stm_ptr->detectorLogic.reset();
//		      sprintf(cbuffer, "Trolley detected (goal changed), :%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
//		  }
//		  status_string.append(cbuffer);
//		  publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, status_string);

	  }
	  else
	  {
		  agv_detect_pushbutton.hasSolution = false;
		  stm_ptr->detectorLogic.bDetectorHasTarget = false;
	  }


	  agv_detect_pushbutton.east_meters = cmd_east_metres;
	  agv_detect_pushbutton.north_meters = cmd_north_metres;
	  agv_detect_pushbutton.headingDeg = cmd_odom_heading_deg;
	  atc_stm::agv_detector_pub.publish(agv_detect_pushbutton);

	  mtx.unlock();
}


//----------------------------------------------------------------------------------------
bool clickToTurn(atc_msgs::Click_To_Turn::Request  &Req, atc_msgs::Click_To_Turn::Response  &Res)
{
  mtx.lock();
  ROS_WARN("4) ASTM - clickToTurn()...");

  firstCTTLoop = true;
  char cbuffer[200];
  std::string status_string;


  stm_ptr->updateCommandedOdomHeading(Req.cmd_heading_deg);

  atc_stm::MOVEMENT_MODE m_mode = atc_stm::MOVEMENT_MODE::MANUAL_STEER;
  atc_stm::movement_mode = m_mode;
  atc_stm::AGV_STATE a_state = atc_stm::AGV_STATE::CTT_HEADING_HOLD;
  atc_stm::agv_state = a_state;

  Res.movement_mode = atc_stm::movement_mode;
  Res.agv_state = atc_stm::agv_state;

  std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
  std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);

  if(Res.agv_state == 4)
  {
	  sprintf(cbuffer, "ClickToTurn update ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
  }
  else
  {
	  sprintf(cbuffer, "ClickToTurn update NOT ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
  }

  status_string.append(cbuffer);
  Res.status_message = status_string;

  ROS_WARN("%s", status_string.c_str());

  mtx.unlock();
  return true;
}

//----------------------------------------------------------------------------------------
bool navigateToTrolley(atc_msgs::Navigate_To_Trolley::Request  &Req, atc_msgs::Navigate_To_Trolley::Response  &Res)
{
	mtx.lock();

	bNavToTrolley = Req.navigateToTrolley;
	bDetectorGoalSent = false;

	ROS_INFO("navigateToTrolley(%i)...", bNavToTrolley);

	Res.movement_mode = atc_stm::movement_mode;
	Res.agv_state = atc_stm::agv_state;

	char cbuffer[200];
	std::string status_string;
	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);

	if(Res.agv_state == 4)
	{
		sprintf(cbuffer, "NavToTrolley update ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
	}
	else
	{
		sprintf(cbuffer, "NavToTrolley update NOT ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
	}

	status_string.append(cbuffer);
	Res.status_message = status_string;

	status_string.clear();
	sprintf(cbuffer, "Navigating to trolley...");
	status_string.append(cbuffer);
	Res.detector_message = status_string;

	mtx.unlock();
	return true;
}

//----------------------------------------------------------------------------------------
bool dockToTag(atc_msgs::Dock_To_Tag::Request  &Req, atc_msgs::Dock_To_Tag::Response  &Res)
{
	mtx.lock();

	// Mod by Tim (28th July 2022):
	// Set the necessary variables, upon the svc call...
	bDockToAprilTag = Req.dock;
//	atc_stm::movement_mode = atc_stm::MOVEMENT_MODE::WAYPOINT;
//	atc_stm::agv_state = atc_stm::AGV_STATE::PATROL;
	// Reset the AprilTagLogic Variables
	stm_ptr->aprilTagLogic_ptr->resetMarkerPose(0);

	// Set a dummy value, to trigger the "goalChangeDetected()" function
	stm_ptr->aprilTagLogic_ptr->marker_pose_camera_avg.pose.position.x = 5.0;
	stm_ptr->aprilTagLogic_ptr->marker_pose_camera_avg.pose.position.y = 5.0;

	Res.movement_mode = atc_stm::movement_mode;
	Res.agv_state = atc_stm::agv_state;

	char cbuffer[200];
	std::string status_string;
	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);

	ROS_INFO("dockToTag() Curr State:%s, enum:%i, stagingGoalCalc:%i, stagingGoalRea:%i, dockingGoalRea:%i",
			agv_state_string.c_str(), atc_stm::agv_state,
			stm_ptr->aprilTagLogic_ptr->stagingGoalCalculated,
			stm_ptr->aprilTagLogic_ptr->stagingGoalReached,
			stm_ptr->aprilTagLogic_ptr->dockingGoalReached);
	ROS_INFO("	MarkPose_X:%0.2f, Y:%0.2f,   MarkPosePrev_X:%0.2f, Y:%0.2f,",
			stm_ptr->aprilTagLogic_ptr->marker_pose_camera_avg.pose.position.x,
			stm_ptr->aprilTagLogic_ptr->marker_pose_camera_avg.pose.position.y,
			stm_ptr->aprilTagLogic_ptr->marker_pose_camera_avg_previous.pose.position.x,
			stm_ptr->aprilTagLogic_ptr->marker_pose_camera_avg_previous.pose.position.y);

//	if(Res.agv_state == 4)
	if(Res.agv_state == 1)
	{
		sprintf(cbuffer, "DockToTag update ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
	}
	else
	{
		ROS_INFO("dockToTag() Current State:%s (supposed to be Patrol!)", atc_utils::getAgvStateString(Res.agv_state));
		sprintf(cbuffer, "DockToTag update NOT ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
	}

	status_string.append(cbuffer);
	Res.status_message = status_string;

	status_string.clear();
	sprintf(cbuffer, "Navigating to trolley...");
	status_string.append(cbuffer);
	Res.dock_message = status_string;

	mtx.unlock();
	return true;
}

//----------------------------------------------------------------------------------------
void stopToStmCallback(const atc_msgs::Stop_To_STM::ConstPtr& stopToStmMsg)
{
	mtx.lock();

	ROS_INFO("stopToStm issued! ");

	atc_stm::MOVEMENT_MODE m_mode = atc_stm::MOVEMENT_MODE::STOP;
	atc_stm::movement_mode = m_mode;
	atc_stm::AGV_STATE a_state = atc_stm::AGV_STATE::STOPPED;
	atc_stm::agv_state = a_state;

	atc_stm::bStopAGV = true;

	// Stop the waypoint logic (if any ongoing)
	std_msgs::Int8 stop_msg;
	stop_msg.data = 1;
	stm_ptr->waypointLogic_ptr->CallbackStop(stop_msg);

//	MoveBaseClient ac("move_base", true);
//    ac.cancelAllGoals();

//    while (!ac.waitForServer(ros::Duration(5.0)))
//    {
//        ROS_INFO("Waiting for the move_base action server to come up...");
//    }

	ROS_INFO("	stopToStm fin ");
    publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Stop to AGV Recieved");

	mtx.unlock();
}

//--------------------------------------------------------------------------------
bool stopGoals()
{
    ROS_INFO("stopGoals() Stopping movement, curr_linear_speed_ms:%.2f, curr_yaw_rate_radsec:%.2f...", curr_linear_speed_ms, curr_yaw_rate_radsec);

    if((fabs(curr_linear_speed_ms) < 0.05) && (fabs(curr_yaw_rate_radsec) < 0.05))
    {
    	ROS_INFO("	currend speeds v small, ignoring...");
        return true;
    }

//    std::string msg = "rosservice call /waypoint_server/stop_wp";
//    system(msg.c_str());

	std_msgs::Int8 stop_msg;
	stop_msg.data = 1;
	stm_ptr->waypointLogic_ptr->CallbackStop(stop_msg);

	// Reset certain variables (in Waypoint Logic!)
//	stm_ptr->detectorLogic.bGoalSent = false;

    ROS_INFO("stopGoals() ok!");

    return true;
}



//--------------------------------------------------------------------------------
bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& goalReached)
{
    ROS_INFO("sendGoal()");

//    ROS_INFO("	Clearing AprilTag Instance");
//	stm_ptr->aprilTagLogic_ptr->reset();

    move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = nav_goal_pose.header.frame_id;
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose = nav_goal_pose;

	//---------------------------------------
	// Debug
	double yawRad = tf::getYaw(goal.target_pose.pose.orientation);

	ROS_INFO(" 	(3-sendGoal())goal pose(x= %.2f y=%.2f yaw:%.2f) frame_id:%s",
			goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
			atc_utils::to_degrees(yawRad), goal.target_pose.header.frame_id.c_str());
	//---------------------------------------

//	run_specific_wp_srv.request.goal = ;
//	specific_wp_client.call(run_specific_wp_srv);

//	atc_msgs::Run_Specific_Wp::Request req;
//	atc_msgs::Run_Specific_Wp::Response res;
//	req.goal = goal;
//	stm_ptr->waypointLogic_ptr->RunSpecificWp(req, res);
//	if (res.success == true)
//	{
//		ROS_INFO("	Goal success");
//		return true;
//	}
//	else
//	{
//		ROS_INFO("	Goal failed");
//		return false;
//	}
//
//	ROS_INFO("	sendGoal() - UNKNOWN");
//	return false;

	MoveBaseClient ac("move_base", true);

	// Assumed was cancel was called b4 this already...
	//ac.cancelAllGoals();

	while (!ac.waitForServer(ros::Duration(5.0)))
	{
	    ROS_INFO("	Waiting for move_base action server to come up...");
	}

	bStopAGV = false;
	bool bFinBeforeTimeout = false;
	do{
		if(!bStopAGV)
		{
			ROS_INFO("	ac.sendGoal()");
			ac.sendGoal(goal);

			ROS_INFO("	ac.waitForResult() waiting...");
			bFinBeforeTimeout = ac.waitForResult();

			ROS_INFO("	ac.waitForResult() finish! ");

			break;
		}
		else
		{
#if DEBUG_CALLBACKS_SENDGOAL
			ROS_INFO("	sendGoal()-Stopped! breaking...");
#endif
			break;
		}
	}while(!bStopAGV);



	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("	sendGoal()-Goal success");
		goalReached = true;
		return true;
	}
	else
	{
		ROS_INFO("	sendGoal()-Goal fail:%s ",  ac.getState().toString().c_str() );
		goalReached = false;
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------
//bool sendSteeringCmds(ros::Publisher& cmd_vel_pub, geometry_msgs::Twist& cmd_velocity_msg)
//{
//
//}

//--------------------------------------------------------------------------------
bool resetClearCostmapCallback(atc_msgs::Reset_ClearCostMap::Request  &Req, atc_msgs::Reset_ClearCostMap::Response  &Res)
{
	mtx.lock();

	ROS_INFO("resetClearCostmapCallback(%i)...", Req.reset_clear_costmap);

	stm_ptr->waypointLogic_ptr->isActive = false;
	stm_ptr->aprilTagLogic_ptr->reset();
	stm_ptr->detectorLogic.reset();

    std::string msg = "rosservice call /move_base/clear_costmaps";
    system(msg.c_str());

	Res.movement_mode = atc_stm::movement_mode;
	Res.agv_state = atc_stm::agv_state;

	char cbuffer[200];
	std::string status_string;
	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);
	sprintf(cbuffer, "Reset & clear costmap ok:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());

	status_string.append(cbuffer);
	Res.status_message = status_string;

	mtx.unlock();
	return true;
}

//--------------------------------------------------------------------------------
//bool chargeAgvCallback(atc_msgs::Charge_AGV::Request  &Req, atc_msgs::Charge_AGV::Response  &Res)
//{
//	mtx.lock();
//
//	ROS_INFO("chargeAgvCallback(%i)...", Req.charge_agv);
//
//	Res.movement_mode = atc_stm::movement_mode;
//	Res.agv_state = atc_stm::agv_state;
//
//	char cbuffer[200];
//	std::string status_string;
//	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
//	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);
//	sprintf(cbuffer, "Charge Agv recieved:%s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
//
//	status_string.append(cbuffer);
//	Res.status_message = status_string;
//
//	mtx.unlock();
//	return true;
//}

//--------------------------------------------------------------------------------
//bool checkGoalStatus(const bool& goalSent, bool& goalReached)
//{
////	if(!goalSent)
////	{
////		ROS_INFO("checkGoalStatus() goal not sent...");
////		goalReached = false;
////		return false;
////	}
//
//	MoveBaseClient ac("move_base", true);
//	while (!ac.waitForServer(ros::Duration(5.0)))
//	{
//	    ROS_INFO("	Waiting for move_base action server to come up...");
//	}
//
//	ROS_INFO("checkGoalStatus()");
//	goalReached = false;
//	if ((ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
//	{
//		ROS_INFO("	Goal success:%s ",  ac.getState().toString().c_str() );
//		goalReached = true;
//		return true;
//	}
//	else if((ac.getState() == actionlib::SimpleClientGoalState::PENDING))
//	{
//		ROS_INFO("	Goal pending:%s ",  ac.getState().toString().c_str() );
//		goalReached = false;
//		return false;
//	}
//	else
//	{
//		ROS_INFO("	Goal failed:%s ",  ac.getState().toString().c_str() );
//		goalReached = false;
//		return false;
//	}
//
//	return false;
//}




} /* namespace atc_stm */
