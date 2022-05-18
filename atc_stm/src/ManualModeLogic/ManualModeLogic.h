/*
 * ManualModeLogic.h
 *
 *  Created on: Apr 16, 2021
 *      Author: timityjoe
 */

#ifndef ATC_MANUALMODELOGIC_H_
#define ATC_MANUALMODELOGIC_H_

#include "ros/ros.h"

namespace atc_stm {

class ManualModeLogic {
public:
	ManualModeLogic();
	virtual ~ManualModeLogic();
	
	double calculateYawRateCommand(const double& P_yaw, const double& MAX_YAW_RATE_RADSEC, bool& hasReached);

	void doClickToTurn(ros::Publisher& cmd_vel_pub);

private:



};

} /* namespace atc_stm */

#endif /* ATC_MANUALMODELOGIC_H_ */
