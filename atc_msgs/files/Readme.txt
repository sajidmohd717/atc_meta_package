// Waypoints
// https://github.com/halejo-as/waypoints
// https://www.youtube.com/watch?v=rPR7Dv711PI&feature=youtu.be
// Create folder /media/Data/workspace/waypoint_ws/src/waypoints/files 
//               /media/Data/workspace/free_fleet/client_ws/install/waypoints/share/waypoints/files/
//else cannot save/load the created configs!
//
// Remember to adjust the topic names in waypoints_server.cpp:
// L188 - MoveBaseClient ac("tb3_0/move_base", true);
// L277 - system("rostopic pub -1 tb3_0/move_base/cancel actionlib_msgs/GoalID -- {} ");
//
rosrun waypoints waypoints_server
roslaunch waypoints wpt_controller.launch
On the panel, select "Load File" then type [mbc_set1] - Refers to both *_gp.txt, *_wp.txt
- Click on 1 waypoint within the panel, then click "Run Group"
