/*
 * AprilTagLogic.cpp
 *
 *  Created on: Apr 6, 2021
 *      Author: timityjoe
 */

#include "AprilTagLogic.h"

#include "../stm_utils.h"
#include "../callbacks.h"

#define DEBUG_APRILTAG_SERVO 1
#define DEBUG_APRILTAG_SERVO_FRONT 0
#define DEBUG_APRILTAG_SERVO_BACK 0
#define DEBUG_APRILTAG_YAW_CMD 0
#define DEBUG_APRILTAG_SPD_CMD 0

namespace atc_stm {

AprilTagLogic::AprilTagLogic(ros::NodeHandle* nh_ptr, std::mutex* mtxPtr) :
    	    //target_frame_("/map"),
    		target_frame_("d435_color_optical_frame"),
			target_frame_front_("d435_front_color_optical_frame"),
//			buffer_back_(ros::Duration(30.0)),
//			buffer_front_(ros::Duration(30.0)),
			buffer_all_(ros::Duration(30.0)),
//			tf2_listener_back_(buffer_back_),
//			tf2_listener_front_(buffer_front_),
			tf2_listener_all_(buffer_all_),
			tf2_back_filter_(apriltag_back_sub_, buffer_all_, target_frame_, 10, *nh_ptr),
			tf2_front_filter_(apriltag_front_sub_, buffer_all_, target_frame_front_, 10, *nh_ptr),
			mtx_ptr(mtxPtr),
    		stagingGoalReached(false),
			stagingGoalCalculated(false),
			dockingGoalReached(false),
			isBackEmpty(false),
			isFrontEmpty(false),
			bIsTrolleyFront(false),
			closestTrolleyID(-1)
{
	ROS_INFO("AprilTagLogic::()");
    // ros::NodeHandle nh_private("~");

	// Back Camera
	apriltag_back_sub_.subscribe(*nh_ptr, "/tag_detections", 1);
    tf2_back_filter_.registerCallback(boost::bind(&AprilTagLogic::aprilTagPoseBackCb, this, _1));

    // Front Camera
	apriltag_front_sub_.subscribe(*nh_ptr, "/tag_detections_front", 1);
    tf2_front_filter_.registerCallback(boost::bind(&AprilTagLogic::aprilTagPoseFrontCb, this, _1));


    //nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    //nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("move_base/current_goal", 1);
    nh_ptr->param("offset_x", offset_x_, 2.0);
    nh_ptr->param("offset_z", offset_z_, 0.0);
    nh_ptr->param("offset_yaw", offset_yaw_, 0.0);

    bFirstAprilTagLoop = true;
}

//-----------------------------------------------------------------------------
AprilTagLogic::~AprilTagLogic() {

}

//-----------------------------------------------------------------------------
void AprilTagLogic::reset()
{
//#if DEBUG_APRILTAG_SERVO
//	  ROS_INFO("AprilTagLogic::reset() ");
//#endif

	stagingGoalCalculated = false;

	stagingGoalReached = false;
	dockingGoalReached = false;
	bDockToAprilTag = false;
	bFirstAprilTagLoop = true;

	marker_pose_camera_avg.pose.position.x = 0.0;
	marker_pose_camera_avg.pose.position.y = 0.0;
	marker_pose_camera_avg.pose.orientation.x = 0.0;
	marker_pose_camera_avg.pose.orientation.y = 0.0;
	marker_pose_camera_avg.pose.orientation.z = 0.0;
	marker_pose_camera_avg.pose.orientation.w = 0.0;

	marker_pose_vec_.clear();
}

//--------------------------------------------------------------------------------
void AprilTagLogic::notifyGroundStation(const bool& hasSolution, const int& closestTagID)
{
#if DEBUG_APRILTAG_SERVO
//	  ROS_INFO("AprilTagLogic::notifyGroundStation() ");
#endif

	  // Send out the notification to rviz (for pushbutton)
	  // Reciprocal (hasSolution = true) is in main()
	  atc_msgs::AprilTag_Has_Solution agv_docking_pushbutton;
	  agv_docking_pushbutton.hasSolution = hasSolution;
	  agv_docking_pushbutton.tagID = closestTagID;
	  atc_stm::agv_docking_pub.publish(agv_docking_pushbutton);
}

//--------------------------------------------------------------------------------
void AprilTagLogic::aprilTagPoseFrontCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
	mtx_ptr->lock();

#if DEBUG_APRILTAG_SERVO_FRONT
	  ROS_INFO("aprilTagPose Front Cb() ");
#endif

    // Mod by Tim:
    const std::string tf_frame_mod = ""+target_frame_front_;


    if (!msg->detections.size())
    {
#if DEBUG_APRILTAG_SERVO
	  //ROS_WARN("	aprilTagPoseFrontCb() 0! msg frame:%s", msg->header.frame_id.c_str());
#endif
      isFrontEmpty = true;
	  mtx_ptr->unlock();
      return;
    }
    else
    {
    	//ROS_INFO("	aprilTagPose Front Cb() detected");
    	isFrontEmpty = false;
    	msgToTagData(msg, tf_frame_mod, false);
    }

	mtx_ptr->unlock();
}


//--------------------------------------------------------------------------------
void AprilTagLogic::aprilTagPoseBackCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
	mtx_ptr->lock();

#if DEBUG_APRILTAG_SERVO_BACK
	  ROS_INFO("aprilTagPose Back Cb() ");
#endif

    // Mod by Tim:
    const std::string tf_frame_mod = ""+target_frame_;


    if (!msg->detections.size())
    {
#if DEBUG_APRILTAG_SERVO
	  //ROS_WARN("	aprilTagPoseBackCb() 0! msg frame:%s", msg->header.frame_id.c_str());
#endif
      isBackEmpty = true;
	  mtx_ptr->unlock();
      return;
    }
    else
    {
    	//ROS_INFO("	aprilTagPose Back Cb() detected");
    	isBackEmpty = false;
    	msgToTagData(msg, tf_frame_mod, true);
    }

	mtx_ptr->unlock();
}

//--------------------------------------------------------------------------------
void AprilTagLogic::msgToTagData(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, const std::string& tgt_frame, const bool fromBackCamera)
{
//	ROS_INFO("msgToTagData() combined_tag_container.size(%i)", combined_tag_container.size());
	for(int i = 0; i < msg->detections.size(); i++)
	{
		TagData tData;
		tData.id            = msg->detections[i].id[0];
		tData.size          = msg->detections[i].size[0];
		tData.pose          = msg->detections[i].pose;
		tData.pixelPosRight = msg->detections[i].pixelPosRight;
		tData.pixelPosDown  = msg->detections[i].pixelPosDown;
		tData.tagArea       = msg->detections[i].tagArea;
		tData.tf_frame_mod  = tgt_frame;
		tData.fromBackCamera = fromBackCamera;

		std::pair<std::map<int,TagData>::iterator,bool> ret;
		ret = combined_tag_container.insert( std::pair<int,TagData>(tData.id,tData) );
		if (ret.second == false)
		{
//			ROS_INFO("msgToTagData() ID%i exists, updating value instead... ", tData.id);
			combined_tag_container[tData.id] = tData;
		}
	}

}


//--------------------------------------------------------------------------------
void AprilTagLogic::periodicSpin()
{
//	mtx_ptr->lock();

//	const std::string tf_frame_mod = ""+target_frame_;

	// Send out the notification to rviz (for pushbutton)
	if(isBackEmpty && isFrontEmpty)
    {
//		ROS_INFO("	periodicSpin() No tags detected! ");
    	notifyGroundStation(false, -1);
    	combined_tag_container.clear();
    	marker_pose_vec_.clear();
    	return;
    }

	// 1) Identify closest tag
	identifyClosestTrolley(combined_tag_container);

	// 1a) Is charging dock..?
	bChargingDock = (closestTrolleyID == CHARGE_TAG_ID) ? (true):(false);

	// 1a) Is trolley front..?
	int remainder = (closestTrolleyID % 2);
	bIsTrolleyFront = ((remainder == 0) && (closestTrolleyID < CHARGE_TAG_ID)) ? (true):(false);

	// 2) Assign target trolley
	assignTargetTrolley(combined_tag_container);


//	mtx_ptr->unlock();
}

//--------------------------------------------------------------------------------
bool AprilTagLogic::identifyClosestTrolley(const std::map<int, TagData>& tags_container)
{
	if(tags_container.size() == 0)
	{
//		ROS_INFO("AprilTagLogic::identifyClosestTrolley() - tags_container.size(%i) == 0! ", tags_container.size());
		closestTrolleyID = -1;
		return false;
	}

	float closestTrolleyTagArea = 0.0;
	int currentClosestTrolleyID = -1;

	std::map<int, TagData>::const_iterator tags_iter;
	for(tags_iter = tags_container.begin(); tags_iter != tags_container.end(); tags_iter++)
	{
		//ROS_INFO("	ID:%i, tagArea:%i", i, msg->detections[i].tagArea);
		if(tags_iter->second.tagArea > closestTrolleyTagArea)
		{
			closestTrolleyTagArea = tags_iter->second.tagArea;
			currentClosestTrolleyID = tags_iter->second.id;
			closestIsfromBackCamera = tags_iter->second.fromBackCamera;
		}
	}

	if((currentClosestTrolleyID != -1) && (currentClosestTrolleyID != closestTrolleyID))
	{
		if(!stagingGoalCalculated)
		{
			closestTrolleyID = currentClosestTrolleyID;
			marker_pose_vec_.clear();
			ROS_INFO("	Change detected! closestTrolley ID:%i", closestTrolleyID);
		}
		else if (((closestTrolleyID+1) == currentClosestTrolleyID) && (stagingGoalCalculated))
		{
			ROS_INFO("	Trolley front tag:%i previously detected, changing to back tag...", closestTrolleyID);
			marker_pose_vec_.clear();
			closestTrolleyID = currentClosestTrolleyID;
		}
		else
		{
			ROS_INFO("	Heading to staging goal: unable to change tgt:%i trolley!", closestTrolleyID);
		}
	}
	else if(currentClosestTrolleyID == closestTrolleyID)
	{
		//ROS_INFO("	NO Change detected.. closestTrolley ID:%i", closestTrolleyID);
	}
	else
	{
		ROS_INFO("	WARN:Unknown branch.. currentClosestTrolleyID:%i, closestTrolley ID:%i", currentClosestTrolleyID, closestTrolleyID);
	}

	//ROS_INFO("	closestTrolley ID:%i, tagArea:%.2f", closestTrolleyID, closestTrolleyTagArea);
	return true;
}

//--------------------------------------------------------------------------------
std::string AprilTagLogic::idToString(const int& ID)
{
	char str[10];
	sprintf(str, "ID%i", ID);

	std::string value(str);
	return value;
}

//--------------------------------------------------------------------------------
bool AprilTagLogic::assignTargetTrolley(const std::map<int, TagData>& tags_container)
{
	atc_stm::debugTagMsg(tags_container, closestTrolleyID, pixelPosRight, pixelPosDown, tagArea);

	// Get mean pose in camera frame
	geometry_msgs::PoseStamped marker_pose_camera_new;

	// Mod by Tim: Have to find within the msg cb..
	std::string tf_frame_mod;
	std::map<int, TagData>::const_iterator tags_iter;
	for(tags_iter = tags_container.begin(); tags_iter != tags_container.end(); tags_iter++)
	{
		if(tags_iter->second.id == closestTrolleyID)
		{
			marker_pose_camera_new.header = tags_iter->second.pose.header;
			tf_frame_mod = tags_iter->second.tf_frame_mod;
			break;
		}
	}

	// Mod by Tim:
	marker_pose_camera_new.header.frame_id = tf_frame_mod;
	atc_stm::populateMarker(tags_container, marker_pose_camera_new, closestTrolleyID);

	notifyGroundStation(true, closestTrolleyID);


	if(bDockToAprilTag)
	{
	    marker_pose_vec_.push_back(marker_pose_camera_new);
		if (marker_pose_vec_.size() == MAX_POSE_COUNT)
		{
			//geometry_msgs::PoseStamped marker_pose_camera_avg;
			marker_pose_camera_avg.header = marker_pose_camera_new.header;
			atc_stm::calculateAverage(marker_pose_vec_, marker_pose_camera_avg);
//			marker_pose_camera_avg_previous = marker_pose_camera_avg;

			// Remove last element, keep the running calculations up to date, also prevent runaway container!
			marker_pose_vec_.pop_back();
			stagingGoalCalculated = true;

#if DEBUG_APRILTAG_SERVO
//      		ROS_INFO("	calculateAverage() done, stagingGoalCalculated...");
#endif
		}
	}
  // determineMotionGoal() previously done here...

	return true;
}

//--------------------------------------------------------------------------------
bool AprilTagLogic::calcMotionGoal(geometry_msgs::PoseStamped& staging_goal_pose)
{
#if DEBUG_APRILTAG_SERVO
//	  ROS_INFO("calcMotionGoal() tagArea:%.2f, closestTrolleyID:%i, isBackEmpty:%i, isFrontEmpty:%i, bIsTrolleyFront:%i",
//			  tagArea, closestTrolleyID, isBackEmpty, isFrontEmpty, bIsTrolleyFront);
#endif

#if DEBUG_APRILTAG_SERVO
//	  ROS_INFO("AprilTagLogic::calcMotionGoal() tagArea:%.2f",tagArea);
#endif

//    const bool hasPoseChanged = atc_stm::poseChangeDetected(marker_pose_camera_avg, marker_pose_camera_);


#if 0
		  ROS_INFO("	Updating marker...AVG_MAP:X1:%.2f, Z1:%.2f   CAM:X2:%.2f, Z2:%.2f",
				marker_pose_camera_avg.pose.position.x, marker_pose_camera_avg.pose.position.z,
				marker_pose_camera_.pose.position.x, marker_pose_camera_.pose.position.z);
#endif
		  geometry_msgs::TransformStamped tf;
		  geometry_msgs::PoseStamped marker_pose_avg_map_;

		  // Mod by Tim:
		  std::string tgtTagFrameTest = idToString(closestTrolleyID);
		  while(!buffer_all_.canTransform("map", tgtTagFrameTest, ros::Time(0), ros::Duration(2.0)))
		  {
			ROS_WARN("calcMotionGoal() canTransform() fail!");
		  }

		  try
		  {
//			  ROS_INFO("	Getting marker_pose_map_...");
//			  buffer_back_.transform(marker_pose_camera_avg, marker_pose_avg_map_, "map");
//			  std::string tgtTagFrame = idToString(closestTrolleyID);
//			  tf = buffer_back_.lookupTransform("map", tgtTagFrame.c_str(), ros::Time(0), ros::Duration(6));
			  ros::Time now = ros::Time::now();
			  if(closestIsfromBackCamera)
			  {
				  //ROS_INFO("calcMotionGoal() closestIsfrom Back Camera...");
			  }
			  else
			  {
				  //ROS_INFO("calcMotionGoal() closestIsfrom Front Camera...");
			  }
				  buffer_all_.transform(marker_pose_camera_avg, marker_pose_avg_map_, "map", ros::Duration(2.0));
				  std::string tgtTagFrame = idToString(closestTrolleyID);
				  //ROS_INFO("failed at second transform");
				  tf = buffer_all_.lookupTransform("map", tgtTagFrame.c_str(), ros::Time(0), ros::Duration(6.0));
				  // tf = buffer_all_.lookupTransform("map",ros::Time(0) + ros::Duration(0.2), tgtTagFrame.c_str(), ros::Time(0), "map", ros::Duration(6.0));

		  }
		  catch (tf2::TransformException &ex)
		  {
			  ROS_WARN("Failure: %s", ex.what());
			  return false;
		  }

		  // Use yaw from tf_echo tools
		  tf2::Quaternion q(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w);
		  tf2::Matrix3x3 m(q);
		  double roll, pitch, yaw;
		  m.getRPY(roll, pitch, yaw);

		  //----------------------------
		  // Debug marker_pose_camera_avg yaw
			double roll1Rad, pitch1Rad, yaw1Rad;
			tf::Quaternion quat_tf1;
			atc_utils::quaternionMsgToTF_no_warn(marker_pose_camera_avg.pose.orientation, quat_tf1);
			atc_utils::quaternion2rpy(quat_tf1, roll1Rad, pitch1Rad, yaw1Rad);
		  //----------------------------

//		  ROS_INFO("	marker_pose_camera_avg x:%.3f, z:%.3f, yawDeg:%.2f frame_id:%s",
//				  marker_pose_camera_avg.pose.position.x, marker_pose_camera_avg.pose.position.z, atc_utils::to_degrees(yaw1Rad), marker_pose_map_.header.frame_id.c_str());
//
//		  ROS_INFO("	marker_pose_map_       x:%.3f, z:%.3f, yawDeg:%.2f frame_id:%s",
//				  marker_pose_map_.pose.position.x, marker_pose_map_.pose.position.z, atc_utils::to_degrees(yaw), marker_pose_map_.header.frame_id.c_str());
//		  ROS_INFO("	roll: %.3f, pitch: %.3f, yaw: %.3f", roll, pitch, yaw);
		  tf2::Quaternion ref_q;

		  const float orientationRadians = (bChargingDock || bIsTrolleyFront) ? (0.0f) : (3.14159f);
//		  ref_q.setRPY(0.0f, 0.0f, yaw + 1.5707f + 3.14159f);
		  ref_q.setRPY(0.0f, 0.0f, yaw + 1.5707f + orientationRadians);

		  // Combine the nav_goal_pose with position from marker_pose_map_,
		  // orientation from lookupTransform between map and Apriltag
//		  geometry_msgs::PoseStamped nav_goal;
//		  offset_x_ = (bIsTrolleyFront) ? (-2.0f) : (2.0f);
		  offset_x_ = (bIsTrolleyFront) ? (-1.2f) : (1.2f);
		  nav_goal_current = marker_pose_avg_map_;
		  nav_goal_current.header.frame_id = "map";
		  nav_goal_current.header.stamp = ros::Time::now();
		  nav_goal_current.pose.position.x -= offset_x_ * cos(yaw + 1.5707f);
		  nav_goal_current.pose.position.y -= offset_x_ * sin(yaw + 1.5707f);
		  nav_goal_current.pose.orientation.x = ref_q.x();
		  nav_goal_current.pose.orientation.y = ref_q.y();
		  nav_goal_current.pose.orientation.z = ref_q.z();
		  nav_goal_current.pose.orientation.w = ref_q.w();

		  staging_goal_pose = nav_goal_current;

		  // Update new goal if changed
//		  const double deltaX = fabs(nav_goal_current.pose.position.x - nav_goal_previous.pose.position.x);
//		  const double deltaY = fabs(nav_goal_current.pose.position.y - nav_goal_previous.pose.position.y);
//		  if ( (deltaX > 0.2) || (deltaY > 0.2) )


		  return true;
}

//--------------------------------------------------------------------------------
bool AprilTagLogic::checkMotionGoalChanged()
{
//	  if(goalChangeDetected(nav_goal_current, nav_goal_previous, tagArea) || bFirstAprilTagLoop)
	  if(goalChangeDetected(marker_pose_camera_avg, marker_pose_camera_avg_previous, tagArea) || bFirstAprilTagLoop)
	  {
#if DEBUG_APRILTAG_SERVO
		ROS_INFO("AprilTagLogic::checkMotionGoalChanged() goal has changed!");
#endif
		return true;
	  }

	  return false;
}

//--------------------------------------------------------------------------------
void AprilTagLogic::sendMotionGoal()
{
#if DEBUG_APRILTAG_SERVO
//		ROS_INFO("AprilTagLogic::sendMotionGoal()");
#endif
		atc_stm::sendGoal(nav_goal_current, stagingGoalReached);
		nav_goal_previous = nav_goal_current;
		marker_pose_camera_avg_previous = marker_pose_camera_avg;

		bFirstAprilTagLoop = false;
}

//--------------------------------------------------------------------------------
// Simple P controller for yaw
// Assume err = 40degs, we wish the
// turn rate to be 0.1radsec
// So P = 0.0025;
// turn rate to be 0.25radsec
// So P = 0.00625;
// turn rate to be 0.5radsec
// So P = 0.0125;
bool AprilTagLogic::calcDockingCmds(double& linearSpdCmd, double& linearYawRateCmd)
{
//	ROS_INFO("AprilTagLogic::calcDockingCmds()");

//	const bool hasTagPoseChanged = atc_stm::tagPoseChangeDetected(marker_pose_camera_avg, marker_pose_camera_);
//	if((!hasTagPoseChanged) && stagingGoalReached && (!dockingGoalReached))
	if(stagingGoalReached && (!dockingGoalReached))
	{
#if DEBUG_APRILTAG_SERVO
//		ROS_INFO("calcDockingCmds() - sending steeering cmds...");
#endif

//		  const double P_linear = 0.0009;
		  const double& MAX_SPEED_METRE_SEC = 0.3;
		  bool hasReached1 = false;
		  const double tagAreaSetPoint = (bChargingDock) ? (180000):(150000);
		  const double P_linear = (bChargingDock) ? (0.0009):(0.0009);
		  linearSpdCmd = calculateLinearSpeedCommand(P_linear, MAX_SPEED_METRE_SEC, tagAreaSetPoint, tagArea, hasReached1);

			// Is charging dock..? Move forward in that case...
		  linearSpdCmd = (bChargingDock) ? (linearSpdCmd * -1.0):(linearSpdCmd);

		  //const double P_yaw = 0.0025;
		  //const double MAX_YAW_RATE_RADSEC = 0.1;
		  //const double P_yaw = 0.00625;
		  //const double MAX_YAW_RATE_RADSEC = 0.25;
		  const double P_yaw = 0.0125;
		  const double MAX_YAW_RATE_RADSEC = 0.5;
		  bool hasReached2 = false;
		  linearYawRateCmd = calculateYawRateCommand(P_yaw, MAX_YAW_RATE_RADSEC, pixelPosRight, hasReached2);

		  // New function, for case where AGV switches from trolley front tag to trolley back tag
		  if(marker_pose_vec_.size() != MAX_POSE_COUNT-1)
		  {
			  ROS_INFO("calcDockingCmds() - Docking state to aquire more data:%i < MAX_POSE_COUNT:%i", marker_pose_vec_.size(), MAX_POSE_COUNT);
			  linearYawRateCmd = 0.0;
			  linearSpdCmd = 0.0;
		  }


		  dockingGoalReached = (hasReached1 && hasReached2) ? (true) : (false);
		  return true;
	}
	else
	{
		// Reset if this condition hits...
#if DEBUG_APRILTAG_SERVO
		ROS_WARN("calcDockingCmds() - unknown state! stagingGoalReached:%i, dockingGoalReached:%i ",
				stagingGoalReached, dockingGoalReached);
#endif
		stagingGoalReached = false;
		dockingGoalReached = false;

		return false;
	}

	return false;
}

//--------------------------------------------------------------------------------
// See vision_servoing/msg/AprilTagDetection.msg
// Tag Area, when near ~4,000, reached 166,000
double AprilTagLogic::calculateLinearSpeedCommand(const double& P_linear, const double& MAX_SPEED_METRE_SEC, const double& tag_area_setpoint, const double& tagArea, bool& hasReached)
{
	//const double tag_area_setpoint = 166000;
	//const double tag_area_setpoint = 170000;
//	const double tag_area_setpoint = 180000;
	const double tag_area_error = -(tag_area_setpoint - tagArea)/1000.0;	// Remove a few (1000) zeroes, for easier calculation

#if DEBUG_APRILTAG_SPD_CMD
	//ROS_INFO("AprilTagLogic::calculateLinearSpeedCommand() tag_area_error:%.2f", tag_area_error);
#endif

	// Simple P controller
	// Assume err = 166K, we wish the
	// turn rate to be 1.0 meter sec
	// So P = 0.0060;
	// turn rate to be 0.6 meter sec
	// So P = 0.0036;
	// turn rate to be 0.3 meter sec
	// So P = 0.0018;
	// turn rate to be 0.15 meter sec
	// So P = 0.0009;
	double cmd_spd_meter_sec = 0;
	if(fabs(tag_area_error) > 5.0)
	{
		cmd_spd_meter_sec = P_linear * tag_area_error;
		hasReached = false;
	}
	else
	{
		cmd_spd_meter_sec = 0;
		hasReached = true;
	}

	// Limit yaw rate
	if(cmd_spd_meter_sec > MAX_SPEED_METRE_SEC)
	{
		cmd_spd_meter_sec = MAX_SPEED_METRE_SEC;
	}
	else if(cmd_spd_meter_sec < -MAX_SPEED_METRE_SEC)
	{
		cmd_spd_meter_sec = -MAX_SPEED_METRE_SEC;
	}

#if DEBUG_APRILTAG_SPD_CMD
	ROS_INFO("	cmd_spd_meter_sec:%.2f", cmd_spd_meter_sec);
#endif

	return cmd_spd_meter_sec;

}

//--------------------------------------------------------------------------------
double AprilTagLogic::calculateYawRateCommand(const double& P_yaw, const double& MAX_YAW_RATE_RADSEC, const double& pixelPosRight, bool& hasReached)
{
	// pixelPosRight 0.0 is left | 0.5 is center | 1.0 is right
	const double boreSightError = 0.5 - pixelPosRight;

	// Intel Realsense d435
	// https://ark.intel.com/content/www/us/en/ark/products/128255/intel-realsense-depth-camera-d435.html
	// Field of View  85.2deg Horiz x 58 deg Vert
	// Resolution     1280 X 720
	// If working in normalized (0.0 - 1.0), then the "gain" is simply 85.2
	const double yaw_error_degs = -boreSightError * 85.2;

#if DEBUG_APRILTAG_YAW_CMD
	ROS_INFO("AprilTagLogic::calculateYawRateCommand() yaw_error_degs:%.2f", yaw_error_degs);
#endif
	// Simple P controller
	// Assume err = 40degs, we wish the
	// turn rate to be 0.1radsec
	// So P = 0.0025;
	// turn rate to be 0.25radsec
	// So P = 0.00625;
	// turn rate to be 0.5radsec
	// So P = 0.0125;
	double cmd_yaw_rate_radsec = 0;
	if(fabs(yaw_error_degs) > 2.0)
	{
		cmd_yaw_rate_radsec = P_yaw * yaw_error_degs;
		hasReached = false;
	}
	else
	{
		cmd_yaw_rate_radsec = 0;
		hasReached = true;
	}

	// Limit yaw rate
	if(cmd_yaw_rate_radsec > MAX_YAW_RATE_RADSEC)
	{
		cmd_yaw_rate_radsec = MAX_YAW_RATE_RADSEC;
	}
	else if(cmd_yaw_rate_radsec < -MAX_YAW_RATE_RADSEC)
	{
		cmd_yaw_rate_radsec = -MAX_YAW_RATE_RADSEC;
	}

#if DEBUG_APRILTAG_YAW_CMD
	ROS_INFO("	cmd_yaw_rate_radsec:%.2f, hasReached:%i", cmd_yaw_rate_radsec, hasReached);
#endif

	return cmd_yaw_rate_radsec;
}




} /* namespace vision_servoing */
