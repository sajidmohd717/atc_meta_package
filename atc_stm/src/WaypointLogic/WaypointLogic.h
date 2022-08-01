/*
 * WaypointLogic.h
 *
 *  Created on: Apr 18, 2021
 *      Author: timityjoe
 */

#ifndef WAYPOINTLOGIC_H_
#define WAYPOINTLOGIC_H_

#include "ros/ros.h"
#include <mutex>

#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

#include "atc_msgs/waypoint_msg.h"
#include "atc_msgs/waypoint_group.h"
#include "atc_msgs/waypointArray.h"
#include "atc_msgs/Save_Wp.h"
#include "atc_msgs/Load_Wp.h"
#include "atc_msgs/Run_Wp.h"
#include "atc_msgs/Stop_Wp.h"
#include "atc_msgs/Delete_Wp.h"
#include "atc_msgs/Groups_Wp.h"
#include "atc_msgs/Wp_2_Group.h"
// *New additions (28th July 2021)
//#include "atc_msgs/Goto_Home.h"
//#include "atc_msgs/Goto_Trolley_DropOff.h"
#include "atc_msgs/RunSpecificWp.h"

// Mod by Tim:
#include "atc_msgs/Run_Specific_Wp.h"

namespace atc_stm {

class WaypointLogic {
public:
	WaypointLogic(ros::NodeHandle* n_ptr, std::mutex* mtxPtr);
	virtual ~WaypointLogic();

	void CallbackStop(std_msgs::Int8 stop_msg);
	// Mod by Tim:
//	bool RunSpecificWp(atc_msgs::Run_Specific_Wp::Request &req, atc_msgs::Run_Specific_Wp::Response &res);

    bool isActive;
    bool doResetAprilTag;

private:
	bool SaveWp(atc_msgs::Save_Wp::Request &req, atc_msgs::Save_Wp::Response &res);
	bool LoadWp(atc_msgs::Load_Wp::Request &req, atc_msgs::Load_Wp::Response &res);

	void CallbackMissionPlan(std_msgs::Int8 mission_plan_msg);
	void CallbackGr(atc_msgs::waypoint_group gr_msg);
	void CallbackWp(atc_msgs::waypoint_msg wp_msg);
	void PublishWp();
	bool RunWp(atc_msgs::Run_Wp::Request &req, atc_msgs::Run_Wp::Response &res);
	bool StopWp(atc_msgs::Stop_Wp::Request &req, atc_msgs::Stop_Wp::Response &res);
	bool DeleteWp(atc_msgs::Delete_Wp::Request &req, atc_msgs::Delete_Wp::Response &res);
	bool GroupOptionWp(atc_msgs::Groups_Wp::Request &req, atc_msgs::Groups_Wp::Response &res);
	bool WpGroup(atc_msgs::Wp_2_Group::Request &req, atc_msgs::Wp_2_Group::Response &res);
//	void onLoop(int state);
//	bool GotoHomeWp(atc_msgs::Goto_Home::Request  &Req, atc_msgs::Goto_Home::Response  &Res);
//	bool GotoTrolleyDropOffWp(atc_msgs::Goto_Trolley_DropOff::Request  &Req, atc_msgs::Goto_Trolley_DropOff::Response  &Res);

	void CallbackRunSpecificWp(atc_msgs::RunSpecificWp wp_name_msg);
	ros::Subscriber sub_specificwp;

    ros::NodeHandle* nh_ptr;
    std::mutex* mtx_ptr;

    //serives
    ros::ServiceServer srv_save;
    ros::ServiceServer srv_load;
    ros::ServiceServer srv_run_wp;
    ros::ServiceServer srv_stop_wp;
    ros::ServiceServer srv_delete_wp;
    ros::ServiceServer srv_groups_wp;
    ros::ServiceServer srv_wp_2_group;

    // *New additions (28th July 2021)
//    ros::ServiceServer srv_gotoHome;
//    ros::ServiceServer srv_gotoTrolleyDropOff;

    // for save/ load files
    std::string path;

    bool stop;


    //internal variables
    std::map<std::string, atc_msgs::waypoint_msg> wp_map;
    std::map<std::string, atc_msgs::waypoint_group> groups;

    ros::Subscriber sub_wp;
    ros::Subscriber sub_gr;
    ros::Publisher pub_wp;
    ros::Publisher pub_gr;
    ros::Publisher pub_debug;

    // Mod by Tim:
    ros::Subscriber sub_mission_plan;
    ros::Subscriber sub_stop;
    //ros::ServiceServer srv_mission_pan;
};

} /* namespace atc_stm */

#endif /* ATC_STM_SRC_WAYPOINTLOGIC_WAYPOINTLOGIC_H_ */
