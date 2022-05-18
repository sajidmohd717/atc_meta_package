/*
 * vision_servoing_utils.h
 *
 *  Created on: Apr 5, 2021
 *      Author: timityjoe
 */

#ifndef VISION_SERVOING_SRC_VISION_SERVOING_UTILS_H_
#define VISION_SERVOING_SRC_VISION_SERVOING_UTILS_H_


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

#include "apriltag_ros/AprilTagDetectionArray.h"



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace atc_stm
{

// Assumes all detections are standalone tags..
class TagData
{
public:
//# Tag ID(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
//# this is a vector containing the IDs of each tag in the bundle.
	int id;
	float size;
	geometry_msgs::PoseWithCovarianceStamped pose;
	float pixelPosRight;
	float pixelPosDown;
	float tagArea;

	std::string tf_frame_mod;
	bool fromBackCamera;
};


//void debugTagMsg(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, double& pixelPosRight, double& pixelPosDown, double& tagArea);
void debugTagMsg(const std::map<int, TagData>& tags_container, const int tgtID, double& pixelPosRight, double& pixelPosDown, double& tagArea);

//void populateMarker(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, geometry_msgs::PoseStamped& marker_pose_camera_new, const int& closestTrolleyID );
void populateMarker(const std::map<int, TagData>& tags_container, geometry_msgs::PoseStamped& marker_pose_camera_new, const int& closestTrolleyID );

void calculateAverage(std::vector<geometry_msgs::PoseStamped>& marker_pose_vec_, geometry_msgs::PoseStamped& marker_pose_camera_avg);

bool goalChangeDetected(const geometry_msgs::PoseStamped& marker_pose_camera_avg, const geometry_msgs::PoseStamped& marker_pose_camera_, const double& tagArea);
bool poseChangeDetected(const geometry_msgs::PoseStamped& marker_pose_camera_avg, const geometry_msgs::PoseStamped& marker_pose_camera_);
bool yawChangeDetected(const geometry_msgs::PoseStamped& marker_pose_camera_avg, const geometry_msgs::PoseStamped& marker_pose_camera_);

//bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& stagingGoalReached);

} /* namespace vision_servoing */

#endif /* VISION_SERVOING_SRC_VISION_SERVOING_UTILS_H_ */
