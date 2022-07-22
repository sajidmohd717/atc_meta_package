/**
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


/* --Includes-- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "atc_utils/angles.h"
#include "atc_utils/atc_utils.h"

// ATC Services
#include "stm_utils.h"

// Callbacks
#include "callbacks.h"

// Logic(s)
//#include "StateTransitionManager.h"
#include "AprilTagLogic/AprilTagLogic.h"
#include "DetectorLogic/DetectorLogic.h"
#include "ManualModeLogic/ManualModeLogic.h"
#include "WaypointLogic/WaypointLogic.h"


#define DEBUG_ATC_STM 0
#define DEBUG_ATC_STM_WAYPOINTMODES 1

//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "atc_stm", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;

  //Setup async spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // AprilTag Logic
  atc_stm::AprilTagLogic aprilTagLogic(&nh, &atc_stm::mtx);

  // Waypoint logic
  atc_stm::WaypointLogic waypointLogic(&nh, &atc_stm::mtx);

  // State transition manager
  atc_stm::StateTransitionManager stm(&atc_stm::mtx, &aprilTagLogic, &waypointLogic);
  atc_stm::stm_ptr = &stm;
  stm.setNodeHandle(nh);

  // Topic Subscribers(s)
  ros::Subscriber imu_sub = nh.subscribe("imu", 2, &atc_stm::imuCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 2, &atc_stm::odomCallback);
  ros::Subscriber amcl_pose_sub   = nh.subscribe("amcl_pose", 2, &atc_stm::amclPoseCallback);
  ros::Subscriber drive_panel_sub = nh.subscribe("drive_panel/cmd_vel", 2, &atc_stm::drivePanelCallback);
  ros::Subscriber stop2stm_sub = nh.subscribe("Stop_To_STM", 2, &atc_stm::stopToStmCallback);

  // Topic Publisher(s)
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);

  // Service Subscribers(s)
  ros::ServiceServer movement_mode_service = nh.advertiseService("atc_stm/Update_Movement_Mode", atc_stm::updateMovementMode);
  ros::ServiceServer ctt_service = nh.advertiseService("atc_stm/Click_To_Turn", atc_stm::clickToTurn);
  ros::ServiceServer navigate2Trolley_service = nh.advertiseService("atc_stm/Navigate_To_Trolley", atc_stm::navigateToTrolley);
  ros::ServiceServer dock2Tag_service = nh.advertiseService("atc_stm/Dock_To_Tag", atc_stm::dockToTag);
  // *New additions (28th July 2021)
  ros::ServiceServer resetClearCostMap_service = nh.advertiseService("atc_stm/Reset_ClearCostMap", atc_stm::resetClearCostmapCallback);
//  ros::ServiceServer chargeAgv_service = nh.advertiseService("atc_stm/Charge_AGV", atc_stm::chargeAgvCallback);

  // Message Subscribers(s)
  ros::Subscriber detector_sub = nh.subscribe<atc_msgs::Detector>("/detection/bounding_boxes", 2, atc_stm::detectorCallback);

  // Message Publishers(s)
  atc_stm::agv_status_pub = nh.advertise<atc_msgs::AGVStatus>("atc_stm/AGVStatus", 1);
  atc_stm::agv_detector_pub = nh.advertise<atc_msgs::Detector_Has_Solution>("atc_stm/Detector_Has_Solution", 1);
  atc_stm::agv_docking_pub = nh.advertise<atc_msgs::AprilTag_Has_Solution>("atc_stm/AprilTag_Has_Solution", 1);

  //  variable to store loop frequency
  int rate(1);
  // Load parameter
  if (nh.hasParam("Custom_Frequency"))
  {
    ROS_INFO("Frequency Parameter available");
    if (nh.getParam("Custom_Frequency", rate))
    {
      ROS_WARN("Updating Frequency:%i hz \n", rate);
    }
  }
  ros::Rate loop_rate(rate);

  // If ROS
  if (!ros::ok())
  {
    ROS_FATAL_STREAM("ROS Node NOT Running..!");
    return -1;
  }

  publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "ATC_STM booted up & online...");

  int count = 0;
  while (ros::ok())
  {
	  // "Spin" apriltag logic
	  stm.aprilTagLogic_ptr->periodicSpin();


	// Calculate the waypoints and steering commands here
//	atc_stm::mtx.lock();
	if((atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::MANUAL_STEER) && (atc_stm::agv_state == atc_stm::AGV_STATE::CTT_HEADING_HOLD))
	{
		stm.waypointLogic_ptr->isActive = false;
		stm.manualModeLogic.doClickToTurn(cmd_vel_pub);
	}
	else if((atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::MANUAL_STEER) && (atc_stm::agv_state == atc_stm::AGV_STATE::JOYSTICK_STEER))
	{
#if DEBUG_ATC_STM
		ROS_INFO("MANUAL_STEER + JOYSTICK_STEER");
#endif

		stm.waypointLogic_ptr->isActive = false;
		stm.aprilTagLogic_ptr->reset();
		stm.detectorLogic.reset();
		cmd_vel_pub.publish(atc_stm::drive_panel_cmd_vel);

	}
	else if(atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::WAYPOINT)
	{
		stm.waypointLogic_ptr->isActive = true;

#if DEBUG_ATC_STM_WAYPOINTMODES
//		ROS_INFO("WAYPOINT");
#endif

		// Trolley detector mode
		if((stm.detectorLogic.bDetectorHasTarget) && (!stm.aprilTagLogic_ptr->stagingGoalCalculated) && atc_stm::bNavToTrolley)
		{
			bool isOk = stm.doWaypointDetectorMode();
			if(!isOk)
			{
				ROS_WARN("	Wpt-Detector mode failed! Going to Stop Mode..");
				atc_stm::movement_mode = atc_stm::MOVEMENT_MODE::STOP;
				atc_stm::agv_state = atc_stm::AGV_STATE::STOPPED;
				// Publish back to Rviz
//				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Wpt-Detector failed!, going to Stop Mode..");
			}
		}
		else if(stm.aprilTagLogic_ptr->stagingGoalCalculated && atc_stm::bDockToAprilTag)	// Docking
		{
			bool isOk = stm.doWaypointDockingMode(cmd_vel_pub);
			if(!isOk)
			{
				ROS_WARN("	Wpt-Docking mode failed! Going to Stop Mode..");
				atc_stm::movement_mode = atc_stm::MOVEMENT_MODE::STOP;
				atc_stm::agv_state = atc_stm::AGV_STATE::STOPPED;
				// Publish back to Rviz
//				publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "Wpt-Docking failed!, going to Stop Mode..");
			}
		}
		else
		{
#if 0
			ROS_INFO("	Normal Wpt Mode: stagingGoalCalculated:%i, bDockToAprilTag:%i ",
					stm.aprilTagLogic_ptr->stagingGoalCalculated, atc_stm::bDockToAprilTag);
#endif
		}

	}
	else if(atc_stm::movement_mode == atc_stm::MOVEMENT_MODE::STOP)
	{
		stm.waypointLogic_ptr->isActive = false;
		stm.aprilTagLogic_ptr->reset();
		stm.detectorLogic.reset();

		// Default values are 0
		geometry_msgs::Twist cmd_velocity_msg;
		cmd_velocity_msg.linear.x = 0.0;
		cmd_velocity_msg.linear.y = 0.0;
		cmd_velocity_msg.linear.z = 0.0;
		cmd_velocity_msg.angular.x = 0.0;
		cmd_velocity_msg.angular.y = 0.0;
		cmd_velocity_msg.angular.z = 0.0;

//		ROS_INFO("STOP");
		cmd_vel_pub.publish(cmd_velocity_msg);

//		publishAgvStatus(atc_stm::movement_mode, atc_stm::agv_state, "AGV Stopped");

	}
	else
	{
#if DEBUG_ATC_STM
		ROS_INFO("UNKNOWN_MODE..!");
#endif
	}
//	atc_stm::mtx.unlock();

	// Remove spin once, since migrating to asyncspinner
    //ros::spinOnce();
	loop_rate.sleep();

	++count;
  }


  return 0;
}


