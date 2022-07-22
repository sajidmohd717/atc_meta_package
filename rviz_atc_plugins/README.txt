

//-----------------------------------------------------------------------------
//		ATC_RVIZ
//-----------------------------------------------------------------------------
cd /media/Data/workspace/atc_rviz_tutorials
sos1
source devel/setup.bash
source $FREE_FLEET/scripts/gazebo_environment.sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
catkin_make

// Run roscore
roscore

//Playback
rosbag play -l mbc.bag		

//Run
rosrun atc_rviz atc_rviz
rosrun rviz rviz

// Check pluginlib plugin_description.xml
rospack plugins --attrib=plugin atc_rviz



//-----------------------------------------------------------------------------
* had to manually edit the /opt/ros/noetic/share/plugin_description.xml
Somehow the atc_rviz package doesnt recognize its own description file ()
  <class name="rviz/ImageTracker" type="rviz::ImageTrackerDisplay" base_class_type="rviz::Display">
    <description>
      Displays an image from a sensor_msgs/Image topic. Additionally mouse click gets published  &lt;a href="http://wiki.ros.org/rviz/DisplayTypes/ImageTracker"&gt;More Information&lt;/a&gt;.
    </description>
    <message_type>sensor_msgs/Image</message_type>
    <message_type>sensor_msgs/CompressedImage</message_type>
    <message_type>theora_image_transport/Packet</message_type>
  </class>
  
//-----------------------------------------------------------------------------
Main classes:
class ImageTrackerPanel: public rviz::Panel
class ImageClickTool: public rviz::Tool
class ImageTrackerDisplay : public ImageDisplayBase
class DriveWidget: public QWidget

//-----------------------------------------------------------------------------


src/pluginlib/include/pluginlib/class_loader_imp.hpp 

src/atc_rviz/viewport_mouse_event.h
src/atc_rviz/render_panel        			- Get mouse events. Returns mouse x, y according to the panel limits
src/atc_rviz/display_factory 
src/atc_rviz/pluginlib_factory 

src/atc_rviz/image/image_display_base  		- On update of ROS messages
src/atc_rviz/image/ROSImageTexture

src/atc_rviz/default_plugins/image_display
src/atc_rviz/default_plugins/image_tracker_display



