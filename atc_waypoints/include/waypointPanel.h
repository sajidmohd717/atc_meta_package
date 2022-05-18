#ifndef WAYPOINTPANEL_H
#define WAYPOINTPANEL_H

#include <ros/ros.h>

#include <rviz/panel.h>
#include <QWidget>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// Mod by Tim
//#include "atc_waypoints/waypoint_msg.h"
//#include "atc_waypoints/waypoint_group.h"
//#include "atc_waypoints/waypointArray.h"
#include "atc_msgs/waypoint_msg.h"
#include "atc_msgs/waypoint_group.h"
#include "atc_msgs/waypointArray.h"




#include <QStringListModel>
#include <map>
#include <list>
namespace Ui
{
    class waypointPanel;
}
namespace atc_waypoints
{

    class waypointPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        explicit waypointPanel(QWidget *parent = 0);
        virtual ~waypointPanel();
        void Callback(atc_msgs::waypointArray wp_msg);
        void PublishWp();
        Ui::waypointPanel *ui;

    protected Q_SLOTS:
        void onDeleteWaypoint();
        void onNewGroup();
        void onDeleteGroup();
        void onAdd2Group();
        void onDelete2Group();
        void onRunGroup();
        void onRunWp();
        void onStopGroup();
        void onGroupBox(const QString &group);
        void onLoop(int state);
        void onSaveFile();
        void onLoadFile();

    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub_rviz;

        std::map<std::string, atc_msgs::waypoint_msg> wp_map;
        std::map<std::string, atc_msgs::waypoint_group> groups;
        std::string textGroupBox = "";
    };

} // end namespace waypoints

#endif // WAIPOINTPANEL_H
