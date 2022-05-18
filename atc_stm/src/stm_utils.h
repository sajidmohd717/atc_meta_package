/*
 * astmutils.h
 *
 *  Created on: Mar 10, 2021
 *      Author: timityjoe
 */

#ifndef SRC_ASTMUTILS_H_
#define SRC_ASTMUTILS_H_


#include <algorithm>
#include <cmath>
#include <math.h>


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "astm_enums.h"

// ATC Services
#include "atc_msgs/Update_Movement_Mode.h"



namespace atc_stm
{
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//------------------------------------------------------------------------
// Variables
extern geometry_msgs::Twist drive_panel_cmd_vel;

//------------------------------------------------------------------------
// Functions
//bool poseEstimatorHasSolution(atc_msgs::PE_Has_Solution::Request  &Req, atc_msgs::PE_Has_Solution::Response  &Res);

void publishAgvStatus(const MOVEMENT_MODE& mode, const AGV_STATE& state, const std::string& status_string);
void publishAgvStatus(const MOVEMENT_MODE& mode, const AGV_STATE& state, const char* status_char);

bool updateMovementMode(atc_msgs::Update_Movement_Mode::Request &Req, atc_msgs::Update_Movement_Mode::Response &Res);

void drivePanelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);

void getCmdGoal(double& cmd_east_m,  double& cmd_north_metres, double& cmd_hdg_deg);

//bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& goalReached);



} /* namespace astm */

#endif /* SRC_ASTMUTILS_H_ */
