
// Via ROS1 (catkin build) vs (catkin make)
https://answers.ros.org/question/214923/roslaunch-cant-find-launch-files-with-catkin-build/

catkin config --no-install
catkin clean --all


// Source ROS1 but colcon make
sos1
source $FREE_FLEET/client_ws/install/setup.bash
source $FREE_FLEET/scripts/gazebo_environment.sh

cd $FREE_FLEET/client_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/noetic/setup.bash
source $FREE_FLEET/client_ws/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE --packages-select [PKG_NAME]



// rosrun topic_tools transform
// https://answers.ros.org/question/232748/subscribe-one-variable-from-big-topic-msg/
// rosmsg show std_msgs/Int32
// rosrun topic_tools transform /test /test/z std_msgs/Float64 'm.z'
rosrun topic_tools transform /tb3_0/amcl_pose /tb3_0/position geometry_msgs/Point 'm.pose.pose.position'
rosrun topic_tools transform /tb3_0/amcl_pose /tb3_0/euler geometry_msgs/Vector3 'tf.transformations.euler_from_quaternion([m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w])' --import tf

rostopic pub /tb3_0/batt_percent std_msgs/Int8 'data: 63' -r 1



// (1) ROS1 to MQTT bridge
// https://github.com/groove-x/mqtt_bridge
// 192.168.8.52:4102
// Topic- /robot1/test/data
sudo apt-get install ros-noetic-rosbridge-library
source $FREE_FLEET/client_ws/install/setup.bash
roslaunch mqtt_bridge demo.launch
roslaunch mqtt_bridge decada_edge.launch

// Run MQTT tests
mosquitto_sub -t '#'
mosquitto_sub -t 'robot1/test/data'
mosquitto_sub -h localhost -p 1883 -t '#'
mosquitto_sub -h 192.168.1.101 -p 1883 -t '#'
mosquitto_sub -h 192.168.8.52 -p 4102 -t 'tb3_0/amcl_pose'
mosquitto_sub -h 192.168.8.52 -p 4102 -t 'tb3_0/amcl_pose/pose/pose/position'
rostopic pub /ping std_msgs/Bool "data: true"
rostopic echo /pong
rostopic pub /echo std_msgs/String "data: 'hello'"
rostopic echo /back
rostopic pub /data_ping std_msgs/Bool "data: true"
rostopic pub -r 10 /data_ping std_msgs/Bool "data: true"


// Launch Free-Fleet Govtech
// * Set inflation_radius: 0.5 [@ /media/Data/workspace/free_fleet/client_ws/src/turtlebot3/turtlebot3_navigation/param/costmap_common_params_burger.yaml]
// Fleet - turtlebot3
// Robot - tb3_0
sos1
source $FREE_FLEET/client_ws/install/setup.bash
source $FREE_FLEET/scripts/gazebo_environment.sh
export TURTLEBOT3_MODEL=burger
roslaunch ff_examples_ros1 single_turtlebot3_ff.mbc.launch	   // MBC single. Makse sure to install ros-noetic-librealsense2
roslaunch ff_examples_ros1 single_turtlebot3_ff.mbc.teb.launch   // MBC single, TEB planner. Makse sure to install ros-noetic-librealsense2

roslaunch ff_examples_ros1 multi_turtlebot3_ff.mbc.launch	// MBC multi
roslaunch turtlebot3_gazebo multi_turtlebot3.mbc.launch	// MBC World file only
roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch	// Original tb3 house
roslaunch ff_examples_ros1 multi_turtlebot3_ff.2.launch	// Original tb3 house, with mqtt/temi interfaces


// Map server
// http://wiki.ros.org/map_server


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

 
// Emulate Decada Dashboard signals (0-2, default is latched)
rostopic pub tb3_0/mission_plan std_msgs/Int8 1 --once
rostopic pub tb3_0/stop std_msgs/Int8 3 --once
rostopic pub tb3_0/mission_plan std_msgs/Int8 2 --once
rosservice call /waypoint_server/run_wp \"wp_name: '' \ngr_name: '"path0"'\n" + "loop: "true"\nindex: "0"\"


// Run Joy - Remember change these settings in /teleop_twist_joy/config/ps3.config.yaml:
// enable_button: 8  # L2 shoulder button
// enable_turbo_button: 10  # L1 shoulder button
// scale_linear_turbo: 1.0
// scale_angular: 0.8
//
// http://wiki.ros.org/joy
// https://github.com/ros-drivers/joystick_drivers.git
roslaunch teleop_twist_joy teleop.launch
roslaunch teleop_twist_joy teleop.tb3_0.launch
roslaunch teleop_twist_joy teleop.caato_0.launch

//-----------------------------------------------------------------------------
//		CAARTO
//-----------------------------------------------------------------------------
sos1
source $FREE_FLEET/client_ws/install/setup.bash
source $FREE_FLEET/scripts/gazebo_environment.sh

colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE --packages-select [PKG_NAME]

conda init bash
conda activate py36 (compiling & runtime)
conda deactivate

roslaunch rviz_atc_plugins  rviz_atc_plugins.launch
roslaunch caato_navigation single_caato_ff.mbc.percept.GDB.launch       // single caato @ MBC, with airport trolley, with perception
roslaunch caato_navigation single_caato_ff.mbc.percept.noGDB.launch       // single caato @ MBC, with airport trolley, with perception

roslaunch caato_navigation single_caato2_ff.mbc.percept.launch      // Updated Caato model

roslaunch caato_navigation single_caato_ff.mbc.percept.noGDB.launch 	// copy packages to dist-pkgs python3 folder. 
									///media/Data/workspace/free_fleet/client_ws/install/pose_estimator
									///share/pose_estimator/config/experiments/exp_group/troll/troll.cfg
roslaunch caato_navigation single_caato_ff.mbc.apriltag.launch     // with Apriltag only
roslaunch caato_navigation single_caato_ff.mbc.launch	      // single caato @ MBC, with airport trolley
roslaunch caato_navigation percept.launch 
roslaunch caato_navigation percept.goicp.launch 
roslaunch caato_navigation generate_goal.launch    
rosrun    caato_navigation april_tag.launch 
rosrun    generate_goal generate_goal_node
roslaunch caato_navigation auto_encoder.launch
roslaunch feature_tracker ft.launch



//-----------------------------------------------------------------------------
// https://github.com/VBot2410/beginner_tutorials
//atc_state_machine_manager
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE --packages-select atc_utils atc_msgs atc_stm rviz_atc_plugins
roslaunch  atc_stm astm.launch
roslaunch  atc_stm astm.GDB.launch
roslaunch  atc_stm astm.noGDB.launch
roslaunch  atc_stm astm.noGDB.2.launch
roslaunch  liftfork_controller liftfork.noGDB.launch

rosrun     atc_stm test_stub
rosservice call /String_Modify Message:"this string to modify"




//-----------------------------------------------------------------------------
// Apriltag
roslaunch vision_servoing apriltag.launch

// Check tf tree
rosrun tf view_frames
dot -Tpdf frames.gv -o frames.pdf

// tf remap
<node pkg="tf" type="tf_remap" name="tf_remapper" output="screen">
  <rosparam param="mappings">
    [{old: depth_camera, new: /camera_rgb_optical_frame},{old: thermal_camera, new: /camera_optris_frame}]
  </rosparam>
</node>




//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Run static transform publisher (add to launch file)
// http://library.isr.ist.utl.pt/docs/roswiki/tf.html#static_transform_publisher
  <!-- Mod by Tim: See http://library.isr.ist.utl.pt/docs/roswiki/tf.html#static_transform_publisher -->
  <!-- node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 map odom 100" / -->
  <node pkg="tf" type="static_transform_publisher" name="base_footprint_broadcaster" args="0 0 0 0 0 0 odom base_footprint 100" />
  <node pkg="tf" type="static_transform_publisher" name="base_link_broadcaster" args="0 0 0 0 0 0 base_footprint base_link 100" />  
  <node pkg="tf" type="static_transform_publisher" name="base_scan_broadcaster" args="0 0 0 0 0 0 base_link base_scan 100" /> 


// Run Mapping
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo mbc.building.nodoors.launch
roslaunch turtlebot3_gazebo mbc.building.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping      
roslaunch turtlebot3_slam turtlebot3_gmapping.launch     
roslaunch turtlebot3_slam turtlebot3_cartographer.launch          
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
rosrun map_server map_saver -f $FREE_FLEET/map

//Run Joy & RQT- Image view
roslaunch teleop_twist_joy teleop.launch

// ROS bag
rosbag record -a -O mbc.bag
rosbag record -O mbc.bag clock scan odom joint_states imu initialpose tf
rosbag info Downloads/mbc.bag
rosbag info Downloads/deutches_museum/*.bag

// Cartographer
// Call the  /media/Data/workspace/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files/turtlebot3_lds_2d_gazebo.lua files
/media/Data/workspace/cartographer_ws
catkin_make_isolated --install --use-ninja

//https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html
// Single bag file (real time, single file playback)
roslaunch cartographer_ros demo_backpack_2d.gazebo.launch bag_filename:=$WORKSPACE/cartographer_ws/Downloads/mbc.bag

// Multiple bag files (Accelerated time, multi file playback)
roslaunch cartographer_ros offline_backpack_2d.gazebo.launch bag_filenames:=$WORKSPACE/cartographer_ws/Downloads/mbc.bag
cartographer_rosbag_validate -bag_filename Downloads/mbc.bag
rosrun tf view_frames
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------------------
// Running [https://github.com/osrf/free_fleet]
source $FREE_FLEET/client_ws/install/setup.bash
roslaunch ff_examples_ros1 fake_client.launch


// Launch Server
ros2 launch ff_examples_ros2 turtlebot3_world_ff.launch.xml

// Launch TB3
source $FREE_FLEET/client_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch ff_examples_ros1 multi_turtlebot3_ff.mbc.launch

roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch
roslaunch ff_examples_ros1 turtlebot3_world_ff.launch



// Intel realsense
  <xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro"/>
  <xacro:include filename="$(find realsense2_description)/urdf/_d435.gazebo.xacro"/>
  <xacro:arg name="use_nominal_extrinsics" default="True" />
  <sensor_d435 parent="base_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </sensor_d435> 


// Gazebo Model check
xmllint ~/model.sdf
gz sdf -k ~/model.sdf


