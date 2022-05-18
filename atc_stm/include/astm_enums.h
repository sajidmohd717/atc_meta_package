/*
 * astm_enums.h
 *
 *  Created on: Mar 3, 2021
 *      Author: timityjoe
 */

#ifndef INCLUDE_ASTM_ENUMS_H_
#define INCLUDE_ASTM_ENUMS_H_

namespace atc_stm {

typedef enum MOVEMENT_MODE // Max 3 modes
{
	UNKNOWN_MODE = 0,
	WAYPOINT,
	MANUAL_STEER,
	STOP
} MOVEMENT_MODE;

typedef enum AGV_STATE	// Max 6 states
{
	UNKNOWN_STATE = 0,
	PATROL,				// Patrol and waypoint is the same (waypoint = patrol movement to 1 waypoint)
	COARSE_GUIDANCE,	// Trolley detected & Pose Estimator has solution
	TERMINAL_GUIDANCE,	// April Tag detected & has solution
	CTT_HEADING_HOLD, 	// Click To Turn (CTT heading hold)
	JOYSTICK_STEER,		// Drive panel widget or joystick cmds
	STOPPED,			// cmd_vel = 0
	E_STOPPED			// E-Stopped means either the local or remote e-stop button is engaged. Cannot disengage from remote panel
} AGV_STATE;


} /* namespace astm */



#endif /* INCLUDE_ASTM_ENUMS_H_ */
