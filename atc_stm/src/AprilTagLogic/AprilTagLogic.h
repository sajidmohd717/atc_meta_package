/*
 * AprilTagLogic.h
 *
 *  Created on: Apr 6, 2021
 *      Author: timityjoe
 */

#ifndef APRILTAGLOGIC_H_
#define APRILTAGLOGIC_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "apriltag_ros/AprilTagDetectionArray.h"

#include "apriltag_utils.h"



namespace atc_stm {

class AprilTagLogic {
public:
	AprilTagLogic(ros::NodeHandle* nh_ptr, std::mutex* mtxPtr);
	virtual ~AprilTagLogic();

	void aprilTagPoseBackCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);
	void aprilTagPoseFrontCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

	bool calcMotionGoal(geometry_msgs::PoseStamped& staging_goal_pose);

	bool checkMotionGoalChanged();
	void sendMotionGoal();

	bool calcDockingCmds(double& linearSpdCmd, double& linearYawRateCmd);

	void msgToTagData(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, const std::string& tgt_frame, const bool fromBackCamera);
	void periodicSpin();

	void reset();
	void resetVariables();

	int getClosestTrolleyID(){return closestTrolleyID;};

	bool stagingGoalCalculated;
	bool stagingGoalReached;
	bool dockingGoalReached;

	// Mod by Tim: To effect goal check upon reaching staging goal
//	bool bFirstAprilTagLoop;
//	int navGoalChangedCounter;

	geometry_msgs::PoseStamped nav_goal_previous;
	geometry_msgs::PoseStamped nav_goal_current;

	geometry_msgs::PoseStamped marker_pose_camera_avg;
	geometry_msgs::PoseStamped marker_pose_camera_avg_previous;
	std::vector<geometry_msgs::PoseStamped> marker_pose_vec_;

private:

	double calculateLinearSpeedCommand(const double& P_linear, const double& MAX_SPEED_METRE_SEC, const double& tag_area_setpoint, const double& tagArea, bool& hasReached);

	double calculateYawRateCommand(const double& P_yaw, const double& MAX_YAW_RATE_RADSEC, const double& pixelPosRight, bool& hasReached);

	void notifyGroundStation(const bool& hasSolution, const int& closestTagID);

//	bool identifyClosestTrolley(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);
	bool identifyClosestTrolley(const std::map<int, TagData>& tags_container);

	std::string idToString(const int& ID);

//	bool assignTargetTrolley(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, const std::string tf_frame_mod);
	bool assignTargetTrolley(const std::map<int, TagData>& tags_container);

//	  tf2_ros::Buffer buffer_back_;
//	  tf2_ros::Buffer buffer_front_;
	  tf2_ros::Buffer buffer_all_;

//	  tf2_ros::TransformListener tf2_listener_back_;
//	  tf2_ros::TransformListener tf2_listener_front_;
	  tf2_ros::TransformListener tf2_listener_all_;

	  // Back Camera
	  bool isBackEmpty;
	  std::string target_frame_;
	  message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> apriltag_back_sub_;
	  tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray> tf2_back_filter_;

	  // Front Camera
	  bool isFrontEmpty;
	  std::string target_frame_front_;
	  message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> apriltag_front_sub_;
	  tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray> tf2_front_filter_;

	  // Combined message
//	  apriltag_ros::AprilTagDetectionArray front_msg;
//	  apriltag_ros::AprilTagDetectionArray back_msg;
	  std::map<int, TagData> combined_tag_container;

	  static constexpr int MAX_POSE_COUNT = 20;
	  double offset_x_, offset_z_, offset_yaw_;

	  // Mod by Tim:
	  std::mutex* mtx_ptr;
	  double pixelPosRight;
	  double pixelPosDown;
	  double tagArea;

	  int closestTrolleyID;
	  bool closestIsfromBackCamera;

	  const int CHARGE_TAG_ID = 15;
	  bool bChargingDock;
	  bool bIsTrolleyFront;


};

} /* namespace vision_servoing */

#endif /* VISION_SERVOING_SRC_APRILTAGSERVO_H_ */
