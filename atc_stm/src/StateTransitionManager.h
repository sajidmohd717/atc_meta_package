/*
 * StateTransitionManager.h
 *
 *  Created on: Mar 3, 2021
 *      Author: timityjoe
 */

#ifndef SRC_StateTransitionManager_H_
#define SRC_StateTransitionManager_H_

#include <mutex>

#include "ros/ros.h"
#include "astm_enums.h"
#include "sensor_msgs/Imu.h"

#include "AprilTagLogic/AprilTagLogic.h"
#include "ManualModeLogic/ManualModeLogic.h"
#include "DetectorLogic/DetectorLogic.h"
#include "WaypointLogic/WaypointLogic.h"

namespace atc_stm {

// TODO:
// Reset conditions
// Lost track conditions
// Re-engage
// Battery charging (docking)

class StateTransitionManager
{
public:
	StateTransitionManager(std::mutex* mtxPtr, AprilTagLogic* aprilTagLogicPtr, WaypointLogic* 	waypointLogicPtr);
	~StateTransitionManager();

	void setNodeHandle(ros::NodeHandle& nh);
//	void reset();

	void updateHeadingAndYawRate (const double& hdg_degs, const double& yaw_rate_degsec);
	void updateCommandedOdomHeading (const double& hdg_degs);

//	void updateOdom(const double& x_m, const double& y_m, const double& odom_hdg_deg, const double& speed_ms);
	void updateLinearSpeed(const double& speed_ms);
	void updateAmclPose(const double& x_m, const double& y_m, const double& odom_hdg_deg);

	bool doWaypointDetectorMode();
	bool doWaypointDockingMode(ros::Publisher& cmd_vel_pub);




	AprilTagLogic* 	aprilTagLogic_ptr;
	WaypointLogic* 	waypointLogic_ptr;
	DetectorLogic 	detectorLogic;
	ManualModeLogic manualModeLogic;


private:
	ros::NodeHandle* nh_ptr;
};

} /* namespace astm */

#endif /* SRC_StateTransitionManager_H_ */
