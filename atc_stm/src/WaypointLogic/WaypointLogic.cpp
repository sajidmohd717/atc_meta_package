/*
 * WaypointLogic.cpp
 *
 *  Created on: Apr 18, 2021
 *      Author: timityjoe
 */

#include "WaypointLogic.h"

#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>

#include "../callbacks.h"


#define foreach BOOST_FOREACH

// Mod by Tim:
#define DEBUG_WPT_SERVER 1

using namespace std;

namespace atc_stm {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//----------------------------------------------------------------------
WaypointLogic::WaypointLogic(ros::NodeHandle* n_ptr, std::mutex* mtxPtr): nh_ptr(n_ptr), mtx_ptr(mtxPtr), isActive(false)
{
    ROS_INFO("WaypointLogic() Constructor");

    path = ros::package::getPath("atc_msgs");

    sub_wp = nh_ptr->subscribe("waypoint", 100, &WaypointLogic::CallbackWp, this);
    sub_gr = nh_ptr->subscribe("waypoint_server/waypoints_gr", 100, &WaypointLogic::CallbackGr, this);

    pub_wp = nh_ptr->advertise<atc_msgs::waypointArray>("waypoint_server/waypoints", 1);
    pub_gr = nh_ptr->advertise<atc_msgs::waypoint_group>("waypoint_server/waypoints_gr", 1);
    pub_debug = nh_ptr->advertise<std_msgs::String>("waypoint_server/statusGoal", 100);
    srv_run_wp = nh_ptr->advertiseService("waypoint_server/run_wp", &WaypointLogic::RunWp, this);

    srv_stop_wp = nh_ptr->advertiseService("waypoint_server/stop_wp", &WaypointLogic::StopWp, this);
    srv_delete_wp = nh_ptr->advertiseService("waypoint_server/delete_wp", &WaypointLogic::DeleteWp, this);
    srv_groups_wp = nh_ptr->advertiseService("waypoint_server/groups_wp", &WaypointLogic::GroupOptionWp, this);
    srv_wp_2_group = nh_ptr->advertiseService("waypoint_server/wp_2_group", &WaypointLogic::WpGroup, this);
    srv_save = nh_ptr->advertiseService("waypoint_server/save_wp", &WaypointLogic::SaveWp, this);
    srv_load = nh_ptr->advertiseService("waypoint_server/load_wp", &WaypointLogic::LoadWp, this);

    // *New additions (28th July 2021)
//    srv_gotoHome = nh_ptr->advertiseService("waypoint_server/Goto_Home", &WaypointLogic::GotoHomeWp, this);
//    srv_gotoTrolleyDropOff = nh_ptr->advertiseService("waypoint_server/Goto_Trolley_DropOff", &WaypointLogic::GotoTrolleyDropOffWp, this);
    sub_specificwp = nh_ptr->subscribe("waypoint_server/RunSpecificWp", 100, &WaypointLogic::CallbackRunSpecificWp, this);

    // Mod by Tim:
    sub_mission_plan = nh_ptr->subscribe("mission_plan", 10, &WaypointLogic::CallbackMissionPlan, this);
    sub_stop = nh_ptr->subscribe("stop", 10, &WaypointLogic::CallbackStop, this);
    //srv_mission_pan = n.advertiseService("mission_plan", &WaypointLogic::CallbackMissionPlan, this);


}


//----------------------------------------------------------------------
WaypointLogic::~WaypointLogic() {

}

//----------------------------------------------------------------------
bool WaypointLogic::SaveWp(atc_msgs::Save_Wp::Request &req, atc_msgs::Save_Wp::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::SaveWp()");
#endif

    std::string pathFile = path + "/files/" + req.file_name + "_wp.txt";
    ofstream file;
    file.open(pathFile, ios::trunc);

    for (auto it = wp_map.begin(); it != wp_map.end(); ++it)
    {
        file << it->second;
        file << "---\n";
    }
    file.close();

    pathFile = path + "/files/" + req.file_name + "_gr.txt";
    file.open(pathFile, ios::trunc);
    for (auto it = groups.begin(); it != groups.end(); ++it)
    {
        file << "name: " << it->second.name.c_str() << "\n";
        file << "wp_list: \n";
        for (auto it_wp : it->second.wp_list)
        {
            file << "   - " << it_wp << "\n";
        }
        file << "---\n";
    }
    file.close();
    ROS_INFO("Save File %s_wp.txt and %s_gr.txt ", req.file_name.c_str(), req.file_name.c_str());
    res.success = true;
    return true;
}

//----------------------------------------------------------------------
bool WaypointLogic::LoadWp(atc_msgs::Load_Wp::Request &req, atc_msgs::Load_Wp::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::LoadWp()");
#endif
    string pathFile = path + "/files/" + req.file_name + "_wp.txt";
    string msg = "rostopic pub -f " + pathFile + " /waypoint atc_msgs/waypoint_msg";
    ROS_INFO("Loaded file %s_wp.txt", req.file_name.c_str());
    ROS_INFO("	%s", msg.c_str());
    system(msg.c_str());

    pathFile = path + "/files/" + req.file_name + "_gr.txt";
    msg = "rostopic pub -f " + pathFile + " /waypoint_server/waypoints_gr atc_msgs/waypoint_group";
    ROS_INFO("Loaded file %s_gr.txt", req.file_name.c_str());
    ROS_INFO("	%s", msg.c_str());
    system(msg.c_str());

    res.success = true;
    return true;
}

//----------------------------------------------------------------------
// Mod by Tim:
void WaypointLogic::CallbackStop(std_msgs::Int8 stop_msg)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::CallbackStop(%i)", stop_msg.data);
#endif
	if(stop_msg.data >= 1)
	{
		atc_msgs::Stop_Wp::Request req;
		atc_msgs::Stop_Wp::Response res;
		bool result = StopWp(req, res);
	}
	else
	{
		ROS_INFO("	Unknown Stop Cmd");
	}

}

//----------------------------------------------------------------------
// Mod by Tim:
void WaypointLogic::CallbackMissionPlan(std_msgs::Int8 mission_plan_msg)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::CallbackMissionPlan(%i)", mission_plan_msg.data);
#endif

	if(mission_plan_msg.data == 1)
	{
		atc_msgs::Run_Wp::Response response;
		atc_msgs::Run_Wp::Request request;
		request.wp_name = "wp1";
		request.gr_name = "path0";
		request.loop = false;
		request.index = 0;

		bool result = RunWp(request, response);
	}
	else if(mission_plan_msg.data == 2)
	{
		atc_msgs::Run_Wp::Response response;
		atc_msgs::Run_Wp::Request request;
		request.wp_name = "wp1_1";
		request.gr_name = "path1";
		request.loop = false;
		request.index = 0;

		bool result = RunWp(request, response);
	}
	else
	{
		ROS_INFO("	Unknown MissionPlan");
	}
}


//----------------------------------------------------------------------
void WaypointLogic::CallbackGr(atc_msgs::waypoint_group gr_msg)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::CallbackGr()");
#endif
    groups[gr_msg.name] = gr_msg;
    ROS_INFO("Received gr %s", gr_msg.name.c_str());
    PublishWp();
}

//----------------------------------------------------------------------
void WaypointLogic::CallbackWp(atc_msgs::waypoint_msg wp_msg)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("WaypointLogic::CallbackWp()");
#endif
    wp_map[wp_msg.name] = wp_msg;
    ROS_INFO("Received wp %s", wp_msg.name.c_str());
    PublishWp();
}

//----------------------------------------------------------------------
void WaypointLogic::PublishWp()
{
    atc_msgs::waypointArray arrayWp;

    //waypoints

    arrayWp.waypoints.resize(wp_map.size());

#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::PublishWp() arrayWp.waypoints.size(%i)", arrayWp.waypoints.size());
#endif

    int i = 0;
    for (auto it = wp_map.begin(); it != wp_map.end(); ++it)
    {
        arrayWp.waypoints[i] = it->second;
        i++;
        ROS_INFO("\n waypoint: %s:\n  Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)\n",
                 it->first.c_str(), it->second.pose.position.x, it->second.pose.position.y, it->second.pose.position.z,
                 it->second.pose.orientation.x, it->second.pose.orientation.y, it->second.pose.orientation.z, it->second.pose.orientation.w);
    }

    //groups
    arrayWp.groups.resize(groups.size());
    i = 0;
    for (auto it = groups.begin(); it != groups.end(); ++it)
    {
        arrayWp.groups[i] = it->second;
        i++;
        //debug
        std::string wp_list = "";
        for (std::string st : it->second.wp_list)
            wp_list = wp_list + st + " ";
        ROS_INFO("group %s: %s \n", it->second.name.c_str(), wp_list.c_str());
    }

    ROS_INFO("Update server");
    pub_wp.publish(arrayWp);
}

//----------------------------------------------------------------------
bool WaypointLogic::RunWp(atc_msgs::Run_Wp::Request &req, atc_msgs::Run_Wp::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("WptLogic::RunWp()");
#endif

	// Mod by Tim: Check if !isActive
	if(!isActive)
	{
		ROS_WARN("WaypointLogic::RunWp() !isActive; set to waypoint mode 1st...? ");
		std_msgs::String msg_debug;
		msg_debug.data = req.wp_name +","+"fail";
		pub_debug.publish(msg_debug);
		return true;
	}

    std::vector<std::string> wp_list = {};
    int i = req.index;

//    atc_stm::mtx_stopAgv.lock();
    stop = 0;
//    bStopAGV = false;
//    atc_stm::mtx_stopAgv.unlock();

    if (req.gr_name.size() > 0)
    {
        wp_list = groups[req.gr_name].wp_list;
    }
    else
    {
        wp_list.push_back(req.wp_name);
    }

    // Mod by Tim:
    //MoveBaseClient ac("move_base", true);
    //MoveBaseClient ac("move_base", true);
    MoveBaseClient ac("move_base", true);

    ac.cancelAllGoals();
    move_base_msgs::MoveBaseGoal goal;
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

#if DEBUG_WPT_SERVER
	ROS_INFO("	[move_base] action server up... ");
#endif

    do
    {
#if DEBUG_WPT_SERVER
	ROS_INFO("	do(), wp_list.size():%i ", wp_list.size());
#endif

        for (i; i < wp_list.size(); i++)
        {
#if DEBUG_WPT_SERVER
        	ROS_INFO("	for(), i:%i ", i);
#endif

            if (!stop)
            {
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = wp_map[wp_list[i]].pose;
                ROS_INFO("	Sending goal:%i ", i);
                ROS_INFO(" 	goal %s, Position (x= %.2f y=%.2f )", req.wp_name.c_str(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                ac.sendGoal(goal);
                ac.waitForResult();
                std::string flag_success = "x";
                std_msgs::String msg_debug;
                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("	WptLogic-Wp:%i:Goal success", i);
                    flag_success = "true";
                    msg_debug.data = req.wp_name +","+"success";

                }
                else
                {
                    ROS_INFO("	WptLogic-Wp:%i:Goal fail:%s ", i, ac.getState().toString().c_str() );
                    flag_success = "false";
                    msg_debug.data = req.wp_name +","+"fail";
                }
                pub_debug.publish(msg_debug);
            }
            else
            {
#if DEBUG_WPT_SERVER
            	ROS_INFO("	WptLogic-Stopped! breaking...");
#endif
                break;
            }
        }
        i = 0;
    } while (!stop && req.loop);

#if DEBUG_WPT_SERVER
	ROS_INFO("	end success \n");
#endif

    return true;
}


//----------------------------------------------------------------------
bool WaypointLogic::StopWp(atc_msgs::Stop_Wp::Request &req, atc_msgs::Stop_Wp::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::StopWp()");
#endif

//	mtx_ptr->lock();
		stop = 1;
		// Mod by Tim: Stop also the other modes/states
		bStopAGV = true;

		// Reset certain variables (in Waypoint Logic!)
		atc_stm::bDetectorGoalSent = false;
		atc_stm::bNavToTrolley = false;
		atc_stm::bDockToAprilTag = false;
//	mtx_ptr->unlock();

    system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID -- {} ");
    res.success = true;

#if DEBUG_WPT_SERVER
	ROS_INFO("	StopWp() ok..");
#endif
    return true;
}
//----------------------------------------------------------------------
bool WaypointLogic::DeleteWp(atc_msgs::Delete_Wp::Request &req, atc_msgs::Delete_Wp::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::DeleteWp()");
#endif
    if (wp_map.find(req.wp_name) == wp_map.end())
    {
        ROS_INFO("Waypoint %s no found", req.wp_name.c_str());
        res.success = false;
    }
    else
    {
        //erase wp of map
        wp_map.erase(req.wp_name);

        //erase wp from all the groups
        if (groups.size() > 0)
        {
            for (auto it = groups.begin(); it != groups.end(); ++it)
            {
                bool flag_rep = 1;
                //erase all the instance of wp
                while (flag_rep)
                {
                    auto it_list = std::find(it->second.wp_list.begin(), it->second.wp_list.end(), req.wp_name);
                    if (it_list != it->second.wp_list.end())
                        it->second.wp_list.erase(it_list);
                    else
                        flag_rep = 0;
                }
            }
        }
        ROS_INFO("Deleted Waypoint %s ", req.wp_name.c_str());
        PublishWp();
        res.success = true;
    }
    return true;
}

//----------------------------------------------------------------------
bool WaypointLogic::GroupOptionWp(atc_msgs::Groups_Wp::Request &req, atc_msgs::Groups_Wp::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::GroupOptionWp()");
#endif
    if (req.option == "add")
    {
        atc_msgs::waypoint_group gr;
        gr.name = req.group_name;
        groups[req.group_name] = gr;
        ROS_INFO("Add new group: %s", req.group_name.c_str());
        res.success = true;
        PublishWp();
    }
    else if (req.option == "delete")
    {
        if (groups.find(req.group_name) == groups.end())
        {
            ROS_INFO("Group %s no found", req.group_name.c_str());
            res.success = false;
        }
        else
        {
            groups.erase(req.group_name);
            ROS_INFO("Deleted group: %s", req.group_name.c_str());
            res.success = true;
            PublishWp();
        }
    }
    else
    {
        ROS_INFO("No valid option (add/delete) supported");
    }
    return true;
}

//----------------------------------------------------------------------
bool WaypointLogic::WpGroup(atc_msgs::Wp_2_Group::Request &req, atc_msgs::Wp_2_Group::Response &res)
{
#if DEBUG_WPT_SERVER
	ROS_INFO("	WptLogic::WpGroup()");
#endif
    if (req.option == "add")
    {
        if (groups.find(req.group_name) == groups.end())
        {
            ROS_INFO("Group %s no found", req.group_name.c_str());
            res.success = false;
        }
        else
        {
            groups[req.group_name].wp_list.push_back(req.wp_name);
            ROS_INFO("Add %s to group %s", req.wp_name.c_str(), req.group_name.c_str());
            res.success = true;
            PublishWp();
        }
    }
    else if (req.option == "delete")
    {
        if (groups.find(req.group_name) == groups.end())
        {
            ROS_INFO("Group %s no found", req.group_name.c_str());
            res.success = false;
        }
        else
        {
            auto it = std::find(groups[req.group_name].wp_list.begin(), groups[req.group_name].wp_list.end(), req.wp_name);
            if (it == groups[req.group_name].wp_list.end())
            {
                ROS_INFO("Waypoint %s no found", req.wp_name.c_str());
                res.success = false;
            }
            else
            {
                if (req.pos > 0)
                {
                    groups[req.group_name].wp_list.erase(groups[req.group_name].wp_list.begin() + req.pos);
                }
                else
                {
                    groups[req.group_name].wp_list.erase(it);
                }

                ROS_INFO("Deleted %s from group %s", req.wp_name.c_str(), req.group_name.c_str());
                res.success = true;
                PublishWp();
            }
        }
    }

    else
    {
        ROS_INFO("No valid option (add/delete) supported");
    }
    return true;
}

//--------------------------------------------------------------------------------
//bool WaypointLogic::GotoHomeWp(atc_msgs::Goto_Home::Request  &Req, atc_msgs::Goto_Home::Response  &Res)
//{
//	ROS_INFO("GotoHomeWp(%i)...", Req.goto_home);
//	Res.movement_mode = atc_stm::movement_mode;
//	Res.agv_state = atc_stm::agv_state;
//
//	char cbuffer[200];
//	std::string status_string;
//	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
//	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);
//	sprintf(cbuffer, "Goto Home recieved: %s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
//
//	status_string.append(cbuffer);
//	Res.status_message = status_string;
//
//	return true;
//}
//
////--------------------------------------------------------------------------------
//bool WaypointLogic::GotoTrolleyDropOffWp(atc_msgs::Goto_Trolley_DropOff::Request  &Req, atc_msgs::Goto_Trolley_DropOff::Response  &Res)
//{
//	ROS_INFO("GotoTrolleyDropOffWp(%i)...", Req.goto_trolley_dropoff);
//	Res.movement_mode = atc_stm::movement_mode;
//	Res.agv_state = atc_stm::agv_state;
//
//	char cbuffer[200];
//	std::string status_string;
//	std::string movement_mode_string = atc_utils::getMovementModeString(atc_stm::movement_mode);
//	std::string agv_state_string = atc_utils::getAgvStateString(atc_stm::agv_state);
//	sprintf(cbuffer, "Goto TrolleyDropOff recieved: %s, %s", movement_mode_string.c_str(), agv_state_string.c_str());
//
//	status_string.append(cbuffer);
//	Res.status_message = status_string;
//
//	return true;
//}

//----------------------------------------------------------------------
// Mod by Tim:
//bool WaypointLogic::runSpecificWp(const std::string& wp_name)
void WaypointLogic::CallbackRunSpecificWp(atc_msgs::RunSpecificWp wp_name_msg)
{
	const std::string wp_name = wp_name_msg.name;

#if DEBUG_WPT_SERVER
	ROS_INFO("WptLogic::CallbackRunSpecificWp(%s)", wp_name.c_str());
#endif

	// Mod by Tim: Check if !isActive
	if(!isActive)
	{
		ROS_WARN("WaypointLogic::CallbackRunSpecificWp() !isActive; set to waypoint mode 1st...? ");
		std_msgs::String msg_debug;
		msg_debug.data = wp_name +","+"fail";
		pub_debug.publish(msg_debug);
		return;
	}

    std::vector<std::string> wp_list = {};

    stop = 0;
    wp_list.push_back(wp_name);

    // Mod by Tim:
    MoveBaseClient ac("move_base", true);

    ac.cancelAllGoals();
    move_base_msgs::MoveBaseGoal goal;
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

#if DEBUG_WPT_SERVER
	ROS_INFO("	[move_base] action server up... ");
#endif

    do
    {
#if DEBUG_WPT_SERVER
	ROS_INFO("	do(), wp_list.size():%i ", wp_list.size());
#endif
		if (!stop)
		{
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			// Mod by Tim: Find the target pose
//			goal.target_pose.pose = wp_map[wp_list[i]].pose;
			bool bWptFound = false;
			std::map<std::string, atc_msgs::waypoint_msg>::iterator mapIter;
			for(mapIter = wp_map.begin(); mapIter != wp_map.end(); mapIter++)
			{
#if DEBUG_WPT_SERVER
				ROS_INFO("	mapIter:%s", mapIter->first.c_str());
#endif

				if(mapIter->first == wp_name)
				{
#if DEBUG_WPT_SERVER
					ROS_INFO("	Wpt:%s found", wp_name.c_str());
#endif
					goal.target_pose.pose = wp_map[wp_name.c_str()].pose;
					bWptFound = true;
					break;
				}
			}

			if(!bWptFound)
			{
				ROS_INFO("	Wpt:%s NOT found in wp_map!", wp_name.c_str());
				return;
			}

			ROS_INFO(" 	Goal %s, Position (x= %.2f y=%.2f )", wp_name.c_str(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			ac.sendGoal(goal);
			ac.waitForResult();
			std::string flag_success = "x";
			std_msgs::String msg_debug;
			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("	Goal success");
				flag_success = "true";
				msg_debug.data = wp_name +","+"success";
				stop = true;

			}
			else
			{
				ROS_INFO("	Goal fail:%s ", ac.getState().toString().c_str() );
				flag_success = "false";
				msg_debug.data = wp_name +","+"fail";
				stop = true;
			}
			pub_debug.publish(msg_debug);
		 }
		 else
		 {
#if DEBUG_WPT_SERVER
			ROS_INFO("	WptLogic-Stopped! breaking...");
#endif
			break;
		 }
    } while (!stop);

#if DEBUG_WPT_SERVER
	ROS_INFO("	end success \n");
#endif

    return;
}



} /* namespace atc_stm */
