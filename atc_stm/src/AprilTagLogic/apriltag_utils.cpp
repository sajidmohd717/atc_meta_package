/*
 * vision_servoing_utils.cpp
 *
 *  Created on: Apr 5, 2021
 *      Author: timityjoe
 */

#include "apriltag_utils.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "atc_utils/atc_utils.h"
#include "atc_utils/angles.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


#define DEBUG_VS_UTILS 0
#define DEBUG_SEND_GOAL 1


namespace atc_stm {

//--------------------------------------------------------------------------------
void debugTagMsg(const std::map<int, TagData>& tags_container, const int tgtID, double& pixelPosRight, double& pixelPosDown, double& tagArea)
{
#if 0
    ROS_INFO("	debugTagSize() tag id:%i, size:%.2f, pxRight:%.2f, pxDown:%.2f, tagArea:%.2f",
    		msg->detections[0].id[0], msg->detections[0].size[0],
			msg->detections[0].pixelPosRight, msg->detections[0].pixelPosDown,
			msg->detections[0].tagArea);
#endif

	std::map<int, TagData>::const_iterator tags_iter;
	for(tags_iter = tags_container.begin(); tags_iter != tags_container.end(); tags_iter++)
	{
		if(tags_iter->second.id == tgtID)
		{
		    pixelPosRight = tags_iter->second.pixelPosRight;
		    pixelPosDown = tags_iter->second.pixelPosDown;
		    tagArea = tags_iter->second.tagArea;
			break;
		}
	}


}

//--------------------------------------------------------------------------------
void populateMarker(const std::map<int, TagData>& tags_container, geometry_msgs::PoseStamped& marker_pose_camera_new, const int& closestTrolleyID)
{
	std::map<int, TagData>::const_iterator tags_iter;
	for(tags_iter = tags_container.begin(); tags_iter != tags_container.end(); tags_iter++)
	{
		if(tags_iter->second.id == closestTrolleyID)
		{
			marker_pose_camera_new.pose.position.x = tags_iter->second.pose.pose.pose.position.x;
			marker_pose_camera_new.pose.position.y = 0; //msg->detections[0].pose.pose.pose.position.y;
			marker_pose_camera_new.pose.position.z = tags_iter->second.pose.pose.pose.position.z;
			marker_pose_camera_new.pose.orientation.x = tags_iter->second.pose.pose.pose.orientation.x;
			marker_pose_camera_new.pose.orientation.y = tags_iter->second.pose.pose.pose.orientation.y;
			marker_pose_camera_new.pose.orientation.z = tags_iter->second.pose.pose.pose.orientation.z;
			marker_pose_camera_new.pose.orientation.w = tags_iter->second.pose.pose.pose.orientation.w;
			break;
		}
	}


#if DEBUG_VS_UTILS
    ROS_INFO("marker pose x:%.3f, y:%.3f",
    		marker_pose_camera_new.pose.position.x, marker_pose_camera_new.pose.position.z);
#endif

}

//--------------------------------------------------------------------------------
void calculateAverage(std::vector<geometry_msgs::PoseStamped>& marker_pose_vec_, geometry_msgs::PoseStamped& marker_pose_camera_avg)
{
	const int MAX_POSE_COUNT = marker_pose_vec_.size();

    double sum_x = 0.0f;
    double sum_z = 0.0f;
    double sum_q_z = 0.0f;
    double sum_q_w = 0.0f;
    for (const auto pose : marker_pose_vec_)
    {
      sum_x += pose.pose.position.x;
      sum_z += pose.pose.position.z;
      sum_q_z += pose.pose.orientation.z;
      sum_q_w += pose.pose.orientation.w;
    }

    double avg_x = sum_x / MAX_POSE_COUNT;
    double avg_z = sum_z / MAX_POSE_COUNT;
    double avg_q_z = sum_q_z / MAX_POSE_COUNT;
    double avg_q_w = sum_q_w / MAX_POSE_COUNT;

    //geometry_msgs::PoseStamped marker_pose_camera_avg;
    //marker_pose_camera_avg.header = marker_pose_camera_new.header;

    // Mod by Tim:
    //marker_pose_camera_avg.header.frame_id = tf_frame_mod;

    // marker_pose_camera_avg.header.stamp = ros::Time::now();
    marker_pose_camera_avg.pose.position.x = avg_x;
    marker_pose_camera_avg.pose.position.z = avg_z;
    marker_pose_camera_avg.pose.orientation.z = avg_q_z;
    marker_pose_camera_avg.pose.orientation.w = avg_q_w;

#if DEBUG_VS_UTILS
    ROS_INFO("	marker pose (camera frame) x:%.3f, y:%.3f, %s",
        marker_pose_camera_avg.pose.position.x, marker_pose_camera_avg.pose.position.z, marker_pose_camera_avg.header.frame_id.c_str());
#endif
}

//--------------------------------------------------------------------------------
bool goalChangeDetected(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2, const double& tagArea)
{
	// Tag Area, when near ~4,000-166,000
	//if(tagArea > 8000)
	if(tagArea > 10000)
	{
		bool bChangeDetected = poseChangeDetected(pose1, pose2, tagArea);

		return bChangeDetected;
	}


	return false;
}

//--------------------------------------------------------------------------------
bool poseChangeDetected(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2, const double& tagArea )
{
//	const bool isYawChanged = yawChangeDetected(pose1, pose2);

	const double deltaX = fabs(pose1.pose.position.x - pose2.pose.position.x);
	const double deltaZ = fabs(pose1.pose.position.z - pose2.pose.position.z);
	const double yawQuatZ = fabs(pose1.pose.orientation.z - pose2.pose.orientation.z);
	const double yawQuatW = fabs(pose1.pose.orientation.w - pose2.pose.orientation.w);

//	ROS_INFO("	poseChangeDetected() deltaX:%.2f, deltaZ:%.2f", deltaX, deltaZ);

//	return(	(deltaX > 0.2f) || (deltaZ > 0.2f) || isYawChanged);
//	const bool isChanged = ((deltaX > 0.2f) || (deltaZ > 0.2f) || (yawQuatZ > 0.05f) || (yawQuatW > 0.05f));
	const bool isChanged = ((deltaX > 0.4f) || (deltaZ > 0.4f) || (yawQuatZ > 0.1f) || (yawQuatW > 0.1f));
//	const bool isChanged = isYawChanged;

	if(isChanged)
	{
		ROS_INFO("	poseChangeDetected() tagArea:%.2f, deltaX:%.2f, Z:%.2f, yawQuatZ:%.2f, W:%.2f", tagArea, deltaX, deltaZ, yawQuatZ, yawQuatW);
	}

	return isChanged;

//	return(	fabs(marker_pose_camera_avg.pose.position.x - marker_pose_camera_.pose.position.x) > 0.2f ||
//	          fabs(marker_pose_camera_avg.pose.position.z - marker_pose_camera_.pose.position.z) > 0.2f ||
//	          fabs(marker_pose_camera_avg.pose.orientation.z - marker_pose_camera_.pose.orientation.z) > 0.05f ||	// 3deg = 0.05rad
//	          fabs(marker_pose_camera_avg.pose.orientation.w - marker_pose_camera_.pose.orientation.w) > 0.05f);
}

//--------------------------------------------------------------------------------
bool yawChangeDetected(const geometry_msgs::PoseStamped& marker_pose_camera_avg, const geometry_msgs::PoseStamped& marker_pose_camera_)
{
	double roll1Rad, pitch1Rad, yaw1Rad;
	tf::Quaternion quat_tf1;
	atc_utils::quaternionMsgToTF_no_warn(marker_pose_camera_avg.pose.orientation, quat_tf1);
	atc_utils::quaternion2rpy(quat_tf1, roll1Rad, pitch1Rad, yaw1Rad);

	double roll2Rad, pitch2Rad, yaw2Rad;
	tf::Quaternion quat_tf2;
	atc_utils::quaternionMsgToTF_no_warn(marker_pose_camera_.pose.orientation, quat_tf2);
	atc_utils::quaternion2rpy(quat_tf2, roll2Rad, pitch2Rad, yaw2Rad);

	const double deltaDegs = atc_utils::to_degrees(atc_utils::shortest_angular_distance(yaw1Rad, yaw2Rad));
	if(deltaDegs > 6.0)
	{
		ROS_INFO("	yawChangeDetected() yaw1deg:%.2f, yaw2deg:%.2f, deltaDegs:%.2f",
				atc_utils::to_degrees(yaw1Rad), atc_utils::to_degrees(yaw2Rad), deltaDegs);
		return true;
	}

	return false;
}


} /* namespace vision_servoing */
