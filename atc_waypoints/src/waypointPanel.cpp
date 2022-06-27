#include "waypointPanel.h"
#include <ui_waypointPanel.h>
#include <vector>
#include <QInputDialog>
#include <QLineEdit>
#include <cstddef>
#include <std_msgs/String.h>
#include <ros/master.h>
#include <thread>

// Mod by Tim:
#define DEBUG_WPT_PANEL 1

namespace atc_waypoints
{
    waypointPanel::waypointPanel(QWidget *parent) : rviz::Panel(parent),
                                                    ui(new Ui::waypointPanel)
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel()");
#endif

        ros::V_string nodes_name;
        ros::master::getNodes(nodes_name);
        bool wp_srv = false;
        for (std::string nh : nodes_name)
        {
        	ROS_INFO("	found node:%s ...", nh.c_str());

        	// Mod by Tim: Waypoints server got subsumed into atc_stm
            //if (nh == "/waypoints_server")
        	if (nh == "/atc_stm")
            {
                ROS_INFO("node %s running", nh.c_str());
                wp_srv = true;
            }
        }
        if (!wp_srv)
        {
            ROS_WARN("node /waypoints_server  is  NOT running!");
        }

        else
        {

            ui->setupUi(this);

            pub_rviz = n.advertise<geometry_msgs::PoseArray>("waypoint_server/waypoints_rviz", 1);
            sub = n.subscribe("waypoint_server/waypoints", 3, &waypointPanel::Callback, this);

            connect(ui->deleteWpButton, SIGNAL(clicked(bool)), this, SLOT(onDeleteWaypoint()));
            connect(ui->newGroupButton, SIGNAL(clicked(bool)), this, SLOT(onNewGroup()));
            connect(ui->deleteGroupButton, SIGNAL(clicked(bool)), this, SLOT(onDeleteGroup()));
            connect(ui->add2GroupButton, SIGNAL(clicked(bool)), this, SLOT(onAdd2Group()));
            connect(ui->delete2GroupButton, SIGNAL(clicked(bool)), this, SLOT(onDelete2Group()));
            connect(ui->stopGroupButton, SIGNAL(clicked(bool)), this, SLOT(onStopGroup()));
            connect(ui->runGroupButton, SIGNAL(clicked(bool)), this, SLOT(onRunGroup()));
            connect(ui->runWpButton, SIGNAL(clicked(bool)), this, SLOT(onRunWp()));
            connect(ui->groupBox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(onGroupBox(const QString &)));
            connect(ui->loopcheckBox, SIGNAL(stateChanged(int)), this, SLOT(onLoop(int)));

            connect(ui->saveFileButton, SIGNAL(clicked(bool)), this, SLOT(onSaveFile()));
            connect(ui->loadFileButton, SIGNAL(clicked(bool)), this, SLOT(onLoadFile()));
        }
    }

    waypointPanel::~waypointPanel()
    {
        delete ui;
    }
    void waypointPanel::PublishWp()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::PublishWp()");
#endif

        geometry_msgs::PoseArray arrayWp_rviz;
        arrayWp_rviz.header.stamp = ros::Time::now();
        arrayWp_rviz.header.frame_id = "map";

        arrayWp_rviz.poses.resize(wp_map.size());
        int i = 0;
        ui->listWp->clear();
        for (auto it = wp_map.begin(); it != wp_map.end(); ++it)
        {
            ui->listWp->addItem(it->first.c_str());
            arrayWp_rviz.poses[i] = it->second.pose;
            i++;
        }
        ui->listGroup->clear();
        ui->groupBox->clear();
        for (auto it = groups.begin(); it != groups.end(); ++it)
        {
            ui->groupBox->addItem(it->first.c_str());
        }
        pub_rviz.publish(arrayWp_rviz);
        int index = ui->groupBox->findText(textGroupBox.c_str());
        ui->groupBox->setCurrentIndex(index);
        ROS_INFO("waypointPanel::PublishWp() Update waypoint server");
    }

    void waypointPanel::Callback(atc_msgs::waypointArray wp_msg)
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::Callback()");
#endif
        wp_map.clear();
        for (atc_msgs::waypoint_msg wp : wp_msg.waypoints)
        {
            wp_map[wp.name] = wp;
        }
#if DEBUG_WPT_PANEL
    	ROS_INFO("	wp_map.size:%i", wp_map.size());
#endif

        groups.clear();
        for (atc_msgs::waypoint_group gr : wp_msg.groups)
        {
            groups[gr.name] = gr;
        }
#if DEBUG_WPT_PANEL
    	ROS_INFO("	groups.size:%i", groups.size());
#endif

        PublishWp();
    }

    void waypointPanel::onDeleteWaypoint()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onDeleteWaypoint()");
#endif
        std::string name = ui->listWp->currentItem()->text().toStdString();
        std::string msg = "rosservice call /waypoint_server/delete_wp \"wp_name: '" + name + "'\"";
        //to not change view
        textGroupBox = ui->groupBox->currentText().toStdString();

        system(msg.c_str());
    }

    void waypointPanel::onNewGroup()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onNewGroup()");
#endif
        std::string name_gp = QInputDialog::getText(nullptr, "Group Name", "Name:", QLineEdit::Normal, "").toStdString();
        if (name_gp.empty())
        {
            ROS_INFO("Put a valid name");
        }
        else
        {
            textGroupBox = ui->groupBox->currentText().toStdString();
            std::string msg = "rosservice call /waypoint_server/groups_wp \"option: 'add' \ngroup_name : '" + name_gp + "'\"";
            system(msg.c_str());
        }
    }

    void waypointPanel::onDeleteGroup()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onDeleteGroup()");
#endif
        if (groups.size() > 0)
        {
            std::string name_gp = ui->groupBox->currentText().toStdString();
            std::string msg = "rosservice call /waypoint_server/groups_wp \"option: 'delete' \ngroup_name : '" + name_gp + "'\"";
            textGroupBox = ui->groupBox->currentText().toStdString();
            system(msg.c_str());
        }
    }

    void waypointPanel::onAdd2Group()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onAdd2Group()");
#endif
        if (ui->listWp->currentRow() != -1 && groups.size() > 0)
        {
            std::string wp = ui->listWp->currentItem()->text().toStdString();
            std::string group = ui->groupBox->currentText().toStdString();
            textGroupBox = group;
            std::string msg = "rosservice call /waypoint_server/wp_2_group \"option: 'add'\ngroup_name: '" + group + "'\nwp_name: '" + wp + "'\"";
            system(msg.c_str());
        }
    }

    void waypointPanel::onDelete2Group()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onDelete2Group()");
#endif
        if (ui->listGroup->currentRow() != -1 && groups.size() > 0)
        {
            std::string wp = ui->listGroup->currentItem()->text().toStdString();
            std::string pos = std::to_string(ui->listGroup->currentRow());
            std::string group = ui->groupBox->currentText().toStdString();
            textGroupBox = group;
            std::string msg = "rosservice call /waypoint_server/wp_2_group \"option: 'delete'\ngroup_name: '" + group + "'\nwp_name: '" + wp + "'\npos: " + pos + "\"";
            system(msg.c_str());
            ROS_INFO("remove wp %s from group %s", wp.c_str(), group.c_str());
        }
    }

    void waypointPanel::onRunGroup()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onRunGroup()");
#endif
        auto run = [](std::string msg) { system(msg.c_str()); };
        if (groups.size() > 0)
        {
            std::string gr_name = ui->groupBox->currentText().toStdString();
            std::string index = std::to_string(ui->listGroup->currentRow());
            bool check = ui->loopcheckBox->isChecked();
            std::string loop;
            if (check)
            {
                ui->runGroupButton->setDisabled(true);
                loop = "true";
            }

            else
                loop = "false";

            std::string msg = "rosservice call /waypoint_server/run_wp \"wp_name: '' \ngr_name: '" + gr_name + "'\n" + "loop: " + loop + "\nindex: " + index + "\"";
            std::thread t1(run, msg);
            t1.detach();

            ROS_INFO("Run Group");
        }
    }

    void waypointPanel::onRunWp()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onRunWp()");
#endif
        auto run = [](std::string msg) { system(msg.c_str()); };
        if (ui->listWp->currentRow() != -1)
        {
            std::string wp = ui->listWp->currentItem()->text().toStdString();
            std::string msg = "rosservice call /waypoint_server/run_wp \"wp_name: '" + wp + "'\ngr_name: ''\nloop: false \nindex: 0 \"";
            std::thread t1(run, msg);
            t1.detach();
            ROS_INFO("Run Wp");
        }
    }

    void waypointPanel::onGroupBox(const QString &group)
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onGroupBox()");
#endif
        ui->listGroup->clear();
        if (groups.size() > 0)
        {
            for (auto it = groups[group.toStdString()].wp_list.begin(); it != groups[group.toStdString()].wp_list.end(); ++it)
            {
                std::string item = *it;
                ui->listGroup->addItem(item.c_str());
            }
        }
    }

    void waypointPanel::onStopGroup()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onStopGroup()");
#endif
        std::string msg = "rosservice call /waypoint_server/stop_wp";
        ui->runGroupButton->setDisabled(false);
        system(msg.c_str());
        ROS_INFO("Stop Movement");
    }

    void waypointPanel::onSaveFile()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onSaveFile()");
#endif
        std::string name_file = QInputDialog::getText(nullptr, "File Name", "Name:", QLineEdit::Normal, "").toStdString();
        if (name_file.empty())
        {
            ROS_INFO("Put a valid name");
        }
        else
        {
            std::string msg = "rosservice call /waypoint_server/save_wp \"file_name: '" + name_file + "'\"";
            system(msg.c_str());
            ROS_INFO("Save File %s_wp.txt and %s_gr.txt ", name_file.c_str(), name_file.c_str());
        }
    }
    void waypointPanel::onLoadFile()
    {
#if DEBUG_WPT_PANEL
    	ROS_INFO("waypointPanel::onLoadFile()");
#endif
        std::string name_file = QInputDialog::getText(nullptr, "File Name", "Name:", QLineEdit::Normal, "").toStdString();
        if (name_file.empty())
        {
            ROS_INFO("Put a valid name");
        }
        else
        {
            std::string msg = "rosservice call /waypoint_server/load_wp \"file_name: '" + name_file + "'\"";
            system(msg.c_str());
            ROS_INFO("Load file %s.txt", name_file.c_str());
        }
    }
    void waypointPanel::onLoop(int state)
    {
    }
} // end namespace waypoints
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(atc_waypoints::waypointPanel, rviz::Panel)
