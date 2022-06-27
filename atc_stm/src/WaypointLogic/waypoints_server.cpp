#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include "atc_waypoints/waypoint_msg.h"
#include "atc_waypoints/waypoint_group.h"
#include "atc_waypoints/waypointArray.h"
#include "atc_waypoints/Save_Wp.h"
#include "atc_waypoints/Load_Wp.h"
#include "atc_waypoints/Run_Wp.h"
#include "atc_waypoints/Stop_Wp.h"
#include "atc_waypoints/Delete_Wp.h"
#include "atc_waypoints/Groups_Wp.h"
#include "atc_waypoints/Wp_2_Group.h"

#include <ros/package.h>

#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>

// Mod by Tim:
#include <std_msgs/Int8.h>


#define foreach BOOST_FOREACH

// Mod by Tim:
#define DEBUG_WPT_SERVER 1

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class WaypointServer
{
public:
    WaypointServer()
    {
        ROS_INFO("WaypointsServer() Constructor");

        path = ros::package::getPath("atc_waypoints");

        sub_wp = n.subscribe("waypoint", 100, &WaypointServer::CallbackWp, this);
        sub_gr = n.subscribe("waypoint_server/waypoints_gr", 100, &WaypointServer::CallbackGr, this);

        pub_wp = n.advertise<atc_waypoints::waypointArray>("waypoint_server/waypoints", 1);
        pub_gr = n.advertise<atc_waypoints::waypoint_group>("waypoint_server/waypoints_gr", 1);
        pub_debug = n.advertise<std_msgs::String>("waypoint_server/statusGoal", 100);
        srv_run_wp = n.advertiseService("waypoint_server/run_wp", &WaypointServer::RunWp, this);
        srv_stop_wp = n.advertiseService("waypoint_server/stop_wp", &WaypointServer::StopWp, this);
        srv_delete_wp = n.advertiseService("waypoint_server/delete_wp", &WaypointServer::DeleteWp, this);
        srv_groups_wp = n.advertiseService("waypoint_server/groups_wp", &WaypointServer::GroupOptionWp, this);
        srv_wp_2_group = n.advertiseService("waypoint_server/wp_2_group", &WaypointServer::WpGroup, this);
        srv_save = n.advertiseService("waypoint_server/save_wp", &WaypointServer::SaveWp, this);
        srv_load = n.advertiseService("waypoint_server/load_wp", &WaypointServer::LoadWp, this);

        // Mod by Tim:
        sub_mission_plan = n.subscribe("mission_plan", 10, &WaypointServer::CallbackMissionPlan, this);
        sub_stop = n.subscribe("stop", 10, &WaypointServer::CallbackStop, this);
        //srv_mission_pan = n.advertiseService("mission_plan", &WaypointServer::CallbackMissionPlan, this);

    }

    bool SaveWp(atc_waypoints::Save_Wp::Request &req, atc_waypoints::Save_Wp::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::SaveWp()");
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

    bool LoadWp(atc_waypoints::Load_Wp::Request &req, atc_waypoints::Load_Wp::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::LoadWp()");
#endif
        string pathFile = path + "/files/" + req.file_name + "_wp.txt";
        string msg = "rostopic pub -f " + pathFile + " /waypoint atc_waypoints/waypoint_msg";
        ROS_INFO("Loaded file %s_wp.txt", req.file_name.c_str());
        ROS_INFO("	%s", msg.c_str());
        system(msg.c_str());

        pathFile = path + "/files/" + req.file_name + "_gr.txt";
        msg = "rostopic pub -f " + pathFile + " /waypoint_server/waypoints_gr atc_waypoints/waypoint_group";
        ROS_INFO("Loaded file %s_gr.txt", req.file_name.c_str());
        ROS_INFO("	%s", msg.c_str());
        system(msg.c_str());

        res.success = true;
        return true;
    }

    //---------------------------------------------------------
    // Mod by Tim:
    void CallbackStop(std_msgs::Int8 stop_msg)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::CallbackStop(%i)", stop_msg);
#endif
    	if(stop_msg.data >= 1)
    	{
    		atc_waypoints::Stop_Wp::Request req;
    		atc_waypoints::Stop_Wp::Response res;
    		bool result = StopWp(req, res);
    	}
    	else
    	{
    		ROS_INFO("	Unknown Stop Cmd");
    	}

    }

    // Mod by Tim:
    void CallbackMissionPlan(std_msgs::Int8 mission_plan_msg)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::CallbackMissionPlan(%i)", mission_plan_msg);
#endif

    	if(mission_plan_msg.data == 1)
    	{
    		atc_waypoints::Run_Wp::Response response;
    		atc_waypoints::Run_Wp::Request request;
    		request.wp_name = "wp1";
			request.gr_name = "path0";
			request.loop = false;
    		request.index = 0;

    		bool result = RunWp(request, response);
    	}
    	else if(mission_plan_msg.data == 2)
    	{
    		atc_waypoints::Run_Wp::Response response;
    		atc_waypoints::Run_Wp::Request request;
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
    //---------------------------------------------------------

    void CallbackGr(atc_waypoints::waypoint_group gr_msg)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::CallbackGr()");
#endif
        groups[gr_msg.name] = gr_msg;
        ROS_INFO("Received gr %s", gr_msg.name.c_str());
        PublishWp();
    }


    void CallbackWp(atc_waypoints::waypoint_msg wp_msg)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("CallbackWp()");
#endif
        wp_map[wp_msg.name] = wp_msg;
        ROS_INFO("Received wp %s", wp_msg.name.c_str());
        PublishWp();
    }

    void PublishWp()
    {
        atc_waypoints::waypointArray arrayWp;

        //waypoints

        arrayWp.waypoints.resize(wp_map.size());

#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::PublishWp() arrayWp.waypoints.size(%i)", arrayWp.waypoints.size());
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

    bool RunWp(atc_waypoints::Run_Wp::Request &req, atc_waypoints::Run_Wp::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::RunWp()");
#endif
        std::vector<std::string> wp_list = {};
        int i = req.index;
        stop = 0;
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
                        ROS_INFO("	Wp:%i:Goal success", i);
                        flag_success = "true";
                        msg_debug.data = req.wp_name +","+"success";
                       
                    }
                    else
                    {
                        ROS_INFO("	Failed to achieved goal:%i ", i);
                        flag_success = "false";
                        msg_debug.data = req.wp_name +","+"fail";
                    }
                    pub_debug.publish(msg_debug);

                    //debug

                    /*char buffer[200];
                    snprintf(buffer, sizeof(buffer), "%f,%s,(x= %.2f y=%.2f ), %s \n",
                             ros::Time::now().toSec(),
                             wp.waypoints[i].name.c_str(),
                             goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
                             flag_success.c_str());

                    std::string line = buffer;
                    std_msgs::String msg;
                    msg.data = line;
                    pub.publish(msg);*/
                }
                else
                {
#if DEBUG_WPT_SERVER
                	ROS_INFO("	stopped! breaking...");
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

    bool StopWp(atc_waypoints::Stop_Wp::Request &req, atc_waypoints::Stop_Wp::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::StopWp()");
#endif
        stop = 1;
        system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID -- {} ");
        res.success = true;
        return true;
    }

    bool DeleteWp(atc_waypoints::Delete_Wp::Request &req, atc_waypoints::Delete_Wp::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::DeleteWp()");
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

    bool GroupOptionWp(atc_waypoints::Groups_Wp::Request &req, atc_waypoints::Groups_Wp::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::GroupOptionWp()");
#endif
        if (req.option == "add")
        {
            atc_waypoints::waypoint_group gr;
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

    bool WpGroup(atc_waypoints::Wp_2_Group::Request &req, atc_waypoints::Wp_2_Group::Response &res)
    {
#if DEBUG_WPT_SERVER
    	ROS_INFO("WaypointsServer::WpGroup()");
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

    void onLoop(int state){

    }

protected:
    ros::NodeHandle n;

    //serives
    ros::ServiceServer srv_save;
    ros::ServiceServer srv_load;
    ros::ServiceServer srv_run_wp;
    ros::ServiceServer srv_stop_wp;
    ros::ServiceServer srv_delete_wp;
    ros::ServiceServer srv_groups_wp;
    ros::ServiceServer srv_wp_2_group;
    // for save/ load files
    std::string path;

    bool stop;

    //internal variables
    std::map<std::string, atc_waypoints::waypoint_msg> wp_map;
    std::map<std::string, atc_waypoints::waypoint_group> groups;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoints_server");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    WaypointServer serv;
    ros::waitForShutdown();
    return 0;
}
