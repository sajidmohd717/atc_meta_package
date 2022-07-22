/*
 * DetectorLogic.h
 *
 *  Created on: Apr 16, 2021
 *      Author: timityjoe
 */

#ifndef DETECTORLOGIC_H_
#define DETECTORLOGIC_H_

#include "../WaypointLogic/WaypointLogic.h"

namespace atc_stm {

class DetectorLogic 
{
public:
	DetectorLogic(WaypointLogic* waypointLogicPtr);
	virtual ~DetectorLogic();
	
	void reset();

	// Used by the patrol modes
//	bool calculateApproximateGoal(const double& boxWidth, const double& boxHeight, const double& pixelPosRight);
	bool calculateApproximateGoal();

	void doNavToTrolley(geometry_msgs::PoseStamped& approx_goal_pose);

	bool bDetectorHasTarget;
//	bool bGoalSent;
	bool bApproxGoalReached;

	double box_width;
	double box_height;
	double box_pixelPosRight;
private:

	double approxTgtRange(const double& boxArea);
	double approxTgtHeading(const double& pixelPosRight, const double& FOV_deg);
	void tgtRangeHeadingToGoal(const double& tgtRangeMeters, const double& tgtHeadingDegs, double& cmd_east_m, double& cmd_north_m);

	WaypointLogic* 	waypointLogic_ptr;
	bool first_detector_Loop;

};

} /* namespace atc_stm */

#endif /* ATC_DETECTORLOGIC_H_ */
