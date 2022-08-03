/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>

#include "drive_widget.h"
#include "image_tracker_panel.h"


// Mod by Tim:
#include "image_tracker_display.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "atc_utils/angles.h"
#include "atc_utils/atc_utils.h"

#define DEBUG_IMG_TRK_PANEL 1


namespace rviz_plugin_tutorials
{

//--------------------------------------------------------------------------------------------
//void ImageTrackerPanel::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
//{
//	std::unique_lock<std::mutex> imu_goal_lock(_imu_mutex);
//
//	double roll, pitch, yaw;
//    atc_utils::imuMessageToRPY(imu_msg, roll, pitch, yaw);
//
//    headingDegs = atc_utils::to_degrees( atc_utils::normalize_angle_positive(-yaw) );
//    char debug_text[150];
//    sprintf (debug_text, "Heading:%.2f Deg", headingDegs);
//    _debug_current_heading_label->setText(debug_text);
//
//    // Update the storage variable
//    rviz::ImageTrackerDisplay::imgTrkInfo.currentHeadingDegrees = headingDegs;
//
//#if DEBUG_IMG_TRK_PANEL
//  //ROS_INFO("imuCallback: headingDegs:%.2f degs",  headingDegs);
//#endif
//}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  east_meters = odom_msg->pose.pose.position.x;
  north_meters = odom_msg->pose.pose.position.y;
  linear_speed_ms = odom_msg->twist.twist.linear.x;

  char debug_text[150];
  sprintf (debug_text, "Speed:%.2f ms", linear_speed_ms);
  _debug_current_linear_spd_label->setText(debug_text);

#if 0
	ROS_INFO("ImageTrackerPanel::odomCallback() x:%.2f, y:%.2f, spd:%.2f ",
			odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->twist.twist.linear.x);
#endif

}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_msg)
{
	  //--------------------------------------------------
	  // Mod by Tim: Change from IMU call back to Odom, for the heading reference..
	  // Odom heading, left  half 0 to 180
	  //               right half 0 t0 -180
	  double rollRad, pitchRad, yawRad;
		tf::Quaternion quat_tf1;
		atc_utils::quaternionMsgToTF_no_warn(amcl_pose_msg->pose.pose.orientation, quat_tf1);
		atc_utils::quaternion2rpy(quat_tf1, rollRad, pitchRad, yawRad);

	//	odom_heading_degs = atc_utils::to_degrees( atc_utils::normalize_angle_positive(-yawRad) );
		odom_heading_degs = atc_utils::to_degrees( yawRad );
	  char debug_text[150];
	  sprintf (debug_text, "Odom_Hdg:%.2f Deg", odom_heading_degs);
	  _debug_current_heading_label->setText(debug_text);

	  // Update the storage variable
	  rviz::ImageTrackerDisplay::imgTrkInfo.currentHeadingDegrees = odom_heading_degs;
	  //--------------------------------------------------
}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::detectorPBCallback(const atc_msgs::Detector_Has_Solution::ConstPtr& detect_pb_msg)
{

	if(detect_pb_msg->hasSolution)
	{
#if 0
	ROS_INFO("ImageTrackerPanel::detectorPBCallback() setting TRUE..");
#endif

		char debug_text[150];
		sprintf (debug_text, "Trolley Detected!");
		_debug_detector_label->setText(debug_text);
		auto_detector_pushButton->setEnabled(true);
	}
	else
	{
		char debug_text[150];
		sprintf (debug_text, "-----");
		_debug_detector_label->setText(debug_text);
		auto_detector_pushButton->setEnabled(false);
	}

}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::dockingPBCallback(const atc_msgs::AprilTag_Has_Solution::ConstPtr& docking_pb_msg)
{

	if(docking_pb_msg->hasSolution)
	{
#if 0
	ROS_INFO("ImageTrackerPanel::dockingPBCallback() setting TRUE..");
#endif

		char debug_text[150];
		if(docking_pb_msg->tagID == 15)
		{
			sprintf (debug_text, "Charge stn:%i Detected!", docking_pb_msg->tagID);
		}
		else
		{
			sprintf (debug_text, "Tag:%i Detected!", docking_pb_msg->tagID);
		}
		_debug_docking_label->setText(debug_text);
		auto_docking_pushButton->setEnabled(true);
	}
	else
	{
		char debug_text[150];
		sprintf (debug_text, "-----");
		_debug_docking_label->setText(debug_text);
		auto_docking_pushButton->setEnabled(false);
	}

}


//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::agvStatusCallback(const atc_msgs::AGVStatus::ConstPtr& agv_status_msg)
{
    char debug_text[150];
    std::string mode_str = atc_utils::getMovementModeString(agv_status_msg->movement_mode);
    sprintf (debug_text, "%s mode set", mode_str.c_str());
    _debug_movement_mode_label->setText(debug_text);

    std::string state_str = atc_utils::getAgvStateString(agv_status_msg->agv_state);
    sprintf (debug_text, "%s state", state_str.c_str());
    _debug_agv_state_label->setText(debug_text);

    sprintf (debug_text, "%s", agv_status_msg->status_message.c_str());
    _debug_status_msg_label->setText(debug_text);
}

//--------------------------------------------------------------------------------------------
//void ImageTrackerPanel::xnergyCallback(const plc_modbus_node::xnergy_sensors::ConstPtr& xnergy_msg)
//{
//	plc_modbus_node::xnergy_sensors xn_sensors = *xnergy_msg;
//
//    char debug_text[200];
////  xn_sensors.xnergy_runtime_voltage;
////	xn_sensors.xnergy_runtime_current;
////	xn_sensors.rcu_temp;
////	xn_sensors.batt_output_current;
////	xn_sensors.battery_volt;
////	xn_sensors.error_code;
//	sprintf (debug_text, "RT_v:%.2f,c:%.2f,t:%i deg, Batt_v:%.2f,c:%.2f, Err_code:%i",
//			xn_sensors.xnergy_runtime_voltage,
//			xn_sensors.xnergy_runtime_current,
//			xn_sensors.rcu_temp,
//			xn_sensors.battery_volt,
//			xn_sensors.batt_output_current,
//			xn_sensors.error_code);
//    _debug_voltage_label->setText(debug_text);
//}

//--------------------------------------------------------------------------------------------
//float32 voltage          # Voltage in Volts (Mandatory)
//float32 current          # Negative when discharging (A)  (If unmeasured NaN)
//float32 charge           # Current charge in Ah  (If unmeasured NaN)
//float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
//float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
//float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
//uint8   power_supply_status     # The charging status as reported. Values defined above
//uint8   power_supply_health     # The battery health metric. Values defined above
//uint8   power_supply_technology # The battery chemistry. Values defined above
//bool    present          # True if the battery is present
void ImageTrackerPanel::xnergyCallback2(const sensor_msgs::BatteryState::ConstPtr& xnergy_msg)
{
	sensor_msgs::BatteryState battState = *xnergy_msg;

    char debug_text[200];
	sprintf (debug_text, "Pct:%.2f, V:%.2f, C:%.2f, Cap:%.2f, S/N:%s",
			battState.percentage,
			battState.voltage,
			battState.current,
			battState.capacity,
			battState.serial_number.c_str());
    _debug_voltage_label->setText(debug_text);
}

//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
	char debug_cmd_heading[50];
	char debug_cmd_linear_spd[50];
	sprintf (debug_cmd_heading, "CmdHeadingSec:%.2f rad/s",
			cmd_vel_msg->angular.z);

	sprintf (debug_cmd_linear_spd, "CmdSpd:%.2f m/s",
			cmd_vel_msg->linear.x);

	_debug_cmd_turnrate_label->setText(debug_cmd_heading);
	_debug_cmd_linear_spd_label->setText(debug_cmd_linear_spd);

}


// ----------------------------------------------------------------------
ImageTrackerPanel::ImageTrackerPanel( QWidget* parent )
  : rviz::Panel( parent )
  , cmd_odom_heading_deg_( 0 )
  , angular_velocity_radsec( 0 ), odom_heading_degs(0.0),
fleet_name_(""), agv_number_("0"), movement_mode(1)
{
  east_meters = 0.0;
  north_meters = 0.0;
  linear_speed_ms = 0.0;

  // Services
  movement_mode_client = nh_.serviceClient<atc_msgs::Update_Movement_Mode>("atc_stm/Update_Movement_Mode");
  ctt_client = nh_.serviceClient<atc_msgs::Click_To_Turn>("atc_stm/Click_To_Turn");
  detector_navigate_client = nh_.serviceClient<atc_msgs::Navigate_To_Trolley>("atc_stm/Navigate_To_Trolley");
  dock_to_tag_client = nh_.serviceClient<atc_msgs::Dock_To_Tag>("atc_stm/Dock_To_Tag");
  // *New additions (28th July 2021)
  reset_clear_costmap_client = nh_.serviceClient<atc_msgs::Reset_ClearCostMap>("atc_stm/Reset_ClearCostMap");
//  goto_home_client = nh_.serviceClient<atc_msgs::Goto_Home>("waypoint_server/Goto_Home");
//  goto_trolley_dropoff_client = nh_.serviceClient<atc_msgs::Goto_Trolley_DropOff>("waypoint_server/Goto_Trolley_DropOff");
  charge_agv_client = nh_.serviceClient<std_srvs::Empty>("xnergy_node/start_charging");
  stop_charge_agv_client = nh_.serviceClient<std_srvs::Empty>("xnergy_node/stop_charging");
  latch_client = nh_.serviceClient<std_srvs::SetBool>("trolley_lifting_arm_srv");


  // Messages in
  agv_state_subscriber = nh_.subscribe("atc_stm/AGVStatus", 2, &ImageTrackerPanel::agvStatusCallback, this);
  agv_detector_pb_subscriber = nh_.subscribe("atc_stm/Detector_Has_Solution", 2, &ImageTrackerPanel::detectorPBCallback, this);
  agv_docking_pb_subscriber = nh_.subscribe("atc_stm/AprilTag_Has_Solution", 2, &ImageTrackerPanel::dockingPBCallback, this);

  // subscribe to xnergy sensors
//  xnergy_subscriber = nh_.subscribe("/modbus/xnergy_sensors", 2, &ImageTrackerPanel::xnergyCallback, this);
  xnergy_subscriber = nh_.subscribe("/xnergy_charger_rcu/battery_state", 2, &ImageTrackerPanel::xnergyCallback2, this);

  // cmd_vel
  cmd_vel_subscriber = nh_.subscribe("/cmd_vel", 2, &ImageTrackerPanel::cmdVelCallback, this);

  // Messages out
  stop_to_stm_publisher_ = nh_.advertise<atc_msgs::Stop_To_STM>("Stop_To_STM", 1 );
  latch_And_changeLaserLimit_publisher_ = nh_.advertise<atc_msgs::Latch>("Latch", 1 );
  run_specific_wp_publisher_ = nh_.advertise<atc_msgs::RunSpecificWp>("waypoint_server/RunSpecificWp", 1 );

  create_topic_box();
  create_status_group_box();
  create_telemetry_group_box();
  create_commands_group_box();
  create_engagements_group_box();
  create_drive_widget_box();

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( _debug_topic_group_box );
  layout->addWidget( _debug_status_group_box );
  layout->addWidget( _debug_telemetry_group_box );
  layout->addWidget( _debug_commands_group_box );
  layout->addWidget( _debug_engagements_group_box );
  layout->addWidget( _debug_drive_widget_box );
  setLayout( layout );

  // Create a timer for sending the output.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setHeadingCmd( float, float )));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendDriveWidgetCmd() ));

  // Mod by Tim:
  connect( fleet_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateFleetName() ));
  connect( agv_number_combobox, SIGNAL( currentTextChanged(QString)), this, SLOT( updateAgvNumber(QString) ));

  // Publish / subscrive to the {fleetname}_{agvNum}/cmd_vel & imu topics
  connect( subscribe_pushButton, SIGNAL( clicked() ), this, SLOT( subscribeAgvTopics() ));

  // Service for movement mode
  connect( movement_mode_combobox, SIGNAL( currentIndexChanged(int)), this, SLOT( updateMovementMode(int) ));
  connect( movement_mode_pushButton, SIGNAL( clicked() ), this, SLOT( sendMovementMode() ));

  // Service for stop to atc_stm
  connect( stop_to_stm_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendStopToSTM(bool) ));

  // Service for navigate to trolley
  connect( auto_detector_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendNavigateToTrolley(bool) ));

  // Service for docking
  connect( auto_docking_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendDockingToTag(bool) ));

  // Service for latching
  connect( latch_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendLatchCommand2(bool) ));

  // *New additions (28th July 2021)
  // Service for reset & clear cost map
  connect( reset_clearCostMap_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendResetClearCostMap(bool) ));
  // Service for Goto Home
  connect( gotoHome_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendGotoHome(bool) ));
  // Service for Goto Trolley DropOff
  connect( gotoTrolley_dropoff_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendGotoTrolleyDropOff(bool) ));
  // Service for wireless charge
  connect( charge_pushButton, SIGNAL( clicked(bool) ), this, SLOT( sendChargeAGV(bool) ));


  // Start the timer.
  output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  drive_widget_->setEnabled( false );

}


//--------------------------------------------------------------------------------------------
//void ImageTrackerPanel::updateMovementMode(QString new_number_string)
void ImageTrackerPanel::updateMovementMode(int index)
{
	movement_mode = index+1;

#if DEBUG_IMG_TRK_PANEL
	//ROS_INFO("ImageTrackerPanel::updateAgvNumber(%s) ", agv_number_combobox->currentText().toStdString().c_str());
	ROS_INFO("ImageTrackerPanel::updateMovementMode(%i) ", movement_mode);
#endif
}

//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::sendMovementMode()
{
	  update_movement_mode_srv.request.movement_mode = movement_mode;

#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendMovementMode(%i) ", update_movement_mode_srv.request.movement_mode);
#endif

	if(!atc_utils::debugServiceCllient(movement_mode_client))
	{
		return;
	}

	  if (movement_mode_client.call(update_movement_mode_srv))
	  {
	    ROS_INFO("	Ok - M Mode:%i, A_State:%i, Status_msg:[%s]",
	    		update_movement_mode_srv.response.movement_mode,
				update_movement_mode_srv.response.agv_state,
				update_movement_mode_srv.response.status_message.c_str());
	    //printMovementMode(update_movement_mode_srv.response.movement_mode);
	    //printAgvState(update_movement_mode_srv.response.agv_state);

	    char debug_text[150];
	    std::string mode_str = atc_utils::getMovementModeString(update_movement_mode_srv.response.movement_mode);
	    sprintf (debug_text, "%s mode set", mode_str.c_str());
	    _debug_movement_mode_label->setText(debug_text);

	    std::string state_str = atc_utils::getAgvStateString(update_movement_mode_srv.response.agv_state);
	    sprintf (debug_text, "%s state", state_str.c_str());
	    _debug_agv_state_label->setText(debug_text);

	    sprintf (debug_text, "%s", update_movement_mode_srv.response.status_message.c_str());
	    _debug_status_msg_label->setText(debug_text);
	  }
	  else
	  {
	    ROS_WARN("	NOK - Failed to call Service [Update_Movement_Mode]");
	  }

}

//--------------------------------------------------------------------------------------------
// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void ImageTrackerPanel::setHeadingCmd( const float& c_heading_deg_, const float& ang_velocity_radsec )
{
#if DEBUG_IMG_TRK_PANEL
	//ROS_INFO("ImageTrackerPanel::setHeadingCmd() ");
#endif

	cmd_odom_heading_deg_ = c_heading_deg_;
  angular_velocity_radsec = ang_velocity_radsec;
}


//--------------------------------------------------------------------------------------------
// Mod by Tim:
void ImageTrackerPanel::updateFleetName()
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::updateFleetName(%s) ", fleet_name_editor_->text().toStdString().c_str());
#endif

	fleet_name_ = fleet_name_editor_->text();
}

//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::updateAgvNumber(QString new_number_string)
{
#if DEBUG_IMG_TRK_PANEL
	//ROS_INFO("ImageTrackerPanel::updateAgvNumber(%s) ", agv_number_combobox->currentText().toStdString().c_str());
	ROS_INFO("ImageTrackerPanel::updateAgvNumber(%s) ", new_number_string.toStdString().c_str());
#endif

	agv_number_ = new_number_string;
}

//--------------------------------------------------------------------------------------------
// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void ImageTrackerPanel::sendDriveWidgetCmd()
{
#if DEBUG_IMG_TRK_PANEL
	//ROS_INFO("ImageTrackerPanel::sendDriveWidgetCmd() ");
#endif

  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = cmd_odom_heading_deg_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_radsec;
    velocity_publisher_.publish( msg );

    char debug_text[150];
    sprintf (debug_text, "CmdHeading:%.2f deg", rviz::ImageTrackerDisplay::imgTrkInfo.commandedHeadingDegrees);
    _debug_cmd_heading_label->setText(debug_text);

    if(rviz::ImageTrackerDisplay::imgTrkInfo.hasChanged)
    {
    	sendCTTHeadingHold(rviz::ImageTrackerDisplay::imgTrkInfo.commandedHeadingDegrees);
    	rviz::ImageTrackerDisplay::imgTrkInfo.hasChanged = false;
    }

  }
}

//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::sendCTTHeadingHold(const double& cmd_heading)
{
	update_ctt_srv.request.cmd_heading_deg = cmd_heading;
//	update_ctt_srv.request.x = 0.0;
//	update_ctt_srv.request.y = 0.0;

#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendCTTHeadingHold(%.2f) ", update_ctt_srv.request.cmd_heading_deg);
#endif

	if(!atc_utils::debugServiceCllient(ctt_client))
	{
		return;
	}

	if (ctt_client.call(update_ctt_srv))
	{
	    ROS_INFO("Ok - M Mode:%i, A_State:%i, Status_msg:[%s]",
	    		update_ctt_srv.response.movement_mode,
				update_ctt_srv.response.agv_state,
				update_ctt_srv.response.status_message.c_str());
	    //printMovementMode(update_movement_mode_srv.response.movement_mode);
	    //printAgvState(update_movement_mode_srv.response.agv_state);

	    char debug_text[150];
	    std::string mode_str = atc_utils::getMovementModeString(update_movement_mode_srv.response.movement_mode);
	    sprintf (debug_text, "%s mode set", mode_str.c_str());
	    _debug_movement_mode_label->setText(debug_text);

	    std::string state_str = atc_utils::getAgvStateString(update_movement_mode_srv.response.agv_state);
	    sprintf (debug_text, "%s state", state_str.c_str());
	    _debug_agv_state_label->setText(debug_text);

	    sprintf (debug_text, "%s", update_movement_mode_srv.response.status_message.c_str());
	    _debug_status_msg_label->setText(debug_text);
	}
	else
	{
	    ROS_WARN("	NOK - Failed to call Service [Click To Turn]");
	}

}

//--------------------------------------------------------------------------------------------
// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ImageTrackerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "FleetName", fleet_name_ );
  config.mapSetValue( "AGVNumber", agv_number_ );
}

//--------------------------------------------------------------------------------------------
// Load all configuration data for this panel from the given Config object.
void ImageTrackerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString fleet_name_topic, agv_number_topic;
  if( config.mapGetString( "FleetName", &fleet_name_topic ))
  {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::load(fleet_name_topic:%s)", fleet_name_topic.toStdString().c_str());
#endif
	  //fleet_name_ = fleet_name_topic;
  }

  if( config.mapGetString( "AGVNumber", &agv_number_topic ))
  {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::load(agv_number_topic:%s)", agv_number_topic.toStdString().c_str());
#endif
	  //agv_number_ = agv_number_topic;
  }

  cmdVelOut_topic_ = getToCmdVelOutTopicString(fleet_name_topic, agv_number_topic);
  imuIn_topic_ = getImuInTopicString(fleet_name_topic, agv_number_topic);
  odomIn_topic_ = getOdomInTopicString(fleet_name_, agv_number_);
  amclPoseIn_topic_ = getAmclPoseInTopicString(fleet_name_, agv_number_);

  setCmdVelOutTopic(cmdVelOut_topic_);
  setImuOdomAmclPoseInTopic(imuIn_topic_, odomIn_topic_, amclPoseIn_topic_);

#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::load(%s : %s)", cmdVelOut_topic_.toStdString().c_str(), imuIn_topic_.toStdString().c_str());
#endif
}

//--------------------------------------------------------------------------------------------
// Set the topic name we are publishing to.
void ImageTrackerPanel::subscribeAgvTopics(void)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::subscribeAgvTopics()");
#endif

	  cmdVelOut_topic_ = getToCmdVelOutTopicString(fleet_name_, agv_number_);
	  imuIn_topic_ = getImuInTopicString(fleet_name_, agv_number_);
	  odomIn_topic_ = getOdomInTopicString(fleet_name_, agv_number_);
	  amclPoseIn_topic_ = getAmclPoseInTopicString(fleet_name_, agv_number_);

#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::subscribeAgvTopics()	(%s : %s)", cmdVelOut_topic_.toStdString().c_str(), imuIn_topic_.toStdString().c_str());
#endif

	  setCmdVelOutTopic(cmdVelOut_topic_);
	  setImuOdomAmclPoseInTopic(imuIn_topic_, odomIn_topic_, amclPoseIn_topic_);

	  //Display
	  char debug_text[150];
	  sprintf (debug_text, "%s | %s", imuIn_topic_.toStdString().c_str(), cmdVelOut_topic_.toStdString().c_str());
	  _debug_fleet_agv_connected_to->setText(debug_text);
}

//--------------------------------------------------------------------------------------------
// Set the topic name we are publishing to.
void ImageTrackerPanel::setCmdVelOutTopic(QString& new_topic )
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::setCmdVelOutTopic(%s) ", new_topic.toStdString().c_str());
#endif

  // Only take action if the name has changed.
//  if( new_topic != cmdVelOut_topic_)
//  {
	  cmdVelOut_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( cmdVelOut_topic_ == "" )
    {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("	vel topic NOT set, shutting vel subscriber down...");
#endif
      velocity_publisher_.shutdown();
    }
    else
    {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("	vel topic %s set, publishing...", cmdVelOut_topic_.toStdString().c_str());
#endif
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( cmdVelOut_topic_.toStdString(), 1 );
    }
    Q_EMIT configChanged();
//  }
//  else
//  {
//#if DEBUG_IMG_TRK_PANEL
//	ROS_INFO("	new_topic == imuIn_topic_ ");
//#endif
//  }

  // Gray out the control widget when the output topic is empty.
  drive_widget_->setEnabled( cmdVelOut_topic_ != "" );
}


//--------------------------------------------------------------------------------------------
// Mod by Tim:
// Set the topic name we are subscribing to.
void ImageTrackerPanel::setImuOdomAmclPoseInTopic(  QString& imuTopic, QString& odomTopic, QString& amclPoseTopic )
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::setImuOdomAmclPoseInTopic(%s) ", imuTopic.toStdString().c_str());
#endif

  // Only take action if the name has changed.
//  if( new_topic != imuIn_topic_ )
//  {
	  imuIn_topic_ = imuTopic;
    // If the topic is the empty string, don't publish anything.
    if( imuIn_topic_ == "" )
    {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("	imu topic NOT set, shutting imu subscriber down...");
#endif
//    	_imu_subscriber.shutdown();
    	agv_odom_subscriber.shutdown();
    	agv_amcl_pose_subscriber.shutdown();
    }
    else
    {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("	imu topic %s set, subscribing...", imuIn_topic_.toStdString().c_str());
#endif

//    	_imu_subscriber = nh_.subscribe(imuIn_topic_.toStdString(), 2, &ImageTrackerPanel::imuCallback, this);
    	agv_odom_subscriber = nh_.subscribe(odomTopic.toStdString(), 2, &ImageTrackerPanel::odomCallback, this);
    	agv_amcl_pose_subscriber = nh_.subscribe(amclPoseTopic.toStdString(), 2, &ImageTrackerPanel::amclPoseCallback, this);
    }
    Q_EMIT configChanged();
//  }
//  else
//  {
//#if DEBUG_IMG_TRK_PANEL
//	ROS_INFO("	new_topic == imuIn_topic_ ");
//#endif
//  }

  // Gray out the control widget when the output topic is empty.
  drive_widget_->setEnabled( imuIn_topic_ != "" );
}

// ----------------------------------------------------------------------
void ImageTrackerPanel::create_topic_box()
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::create_topic_box() ");
#endif

	_debug_topic_group_box = new QGroupBox("Fleet Name & AGV Number");
	//debug_topic_layout = new QHBoxLayout;
	debug_topic_layout = new QGridLayout;

	fleet_name_editor_ = new QLineEdit;
	debug_topic_layout->addWidget( fleet_name_editor_, 1, 0);

	//topic_layout->addWidget( new QLabel( "CmdVel(Out) topic:" ));
	debug_topic_layout->addWidget( new QLabel( "Fleet_Name"), 1, 1 );

	//topic_layout->addWidget( new QLabel( "IMU(In) topic:" ));
//	debug_topic_layout->addWidget( new QLabel( "AGV_Number" ));
//	input_imu_topic_editor_ = new QLineEdit;
//	debug_topic_layout->addWidget( input_imu_topic_editor_ );

	// 1a) Combo box
	agv_number_combobox = new QComboBox;
	agv_number_combobox->setEditable(true);
	agv_number_combobox->lineEdit()->setPlaceholderText("Select AGV Number");
	agv_number_combobox->addItem("0");
	agv_number_combobox->addItem("1");
	agv_number_combobox->addItem("2");
	agv_number_combobox->addItem("3");
	agv_number_combobox->addItem("4");
	agv_number_combobox->addItem("5");
	debug_topic_layout->addWidget(agv_number_combobox, 2, 0);

	// 1b) Send button
	subscribe_pushButton = new QPushButton(tr("&Subscribe"));
	debug_topic_layout->addWidget(subscribe_pushButton, 2, 1);

	// Label to see who we are connected to
	_debug_fleet_agv_connected_to = new QLabel("Connected To:N.A");
	debug_topic_layout->addWidget(_debug_fleet_agv_connected_to, 3, 0);

	_debug_topic_group_box->setLayout(debug_topic_layout);
}

//--------------------------------------------------------------------------------------------
// See https://doc.qt.io/qt-5/qtwidgets-layouts-basiclayouts-example.html
void ImageTrackerPanel::create_status_group_box()
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::create_status_group_box() ");
#endif

  // ---------- Vertical debug labels ----------
  _debug_status_group_box = new QGroupBox("Status/AGV State");

  debug_status_layout = new QGridLayout;

  _debug_status_msg_label = new QLabel("status_msg:N.A");
  debug_status_layout->addWidget(_debug_status_msg_label, 1, 0);

  _debug_agv_state_label = new QLabel("agv_state:N.A");
  debug_status_layout->addWidget(_debug_agv_state_label, 2, 0);

  //debug_status_layout->setRowStretch(1, 30);
  //debug_status_layout->setRowStretch(2, 30);

//  helpEditor = new QTextEdit;
//  helpEditor->setPlainText(tr("This widget takes up about two thirds of the "
//                               "grid layout."));
//  debug_status_layout->addWidget(helpEditor, 1, 1);

  _debug_status_group_box->setLayout(debug_status_layout);
}

 //--------------------------------------------------------------------------------------------
 // See https://doc.qt.io/qt-5/qtwidgets-layouts-basiclayouts-example.html
 void ImageTrackerPanel::create_telemetry_group_box()
 {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::create_telemetry_group_box() ");
#endif
  // ---------- Horizontal debug labels ----------
  _debug_telemetry_group_box = new QGroupBox("Telemetry");
  //debug_telemetry_layout = new QHBoxLayout;
  debug_telemetry_layout = new QGridLayout;

  _debug_current_heading_label = new QLabel("Heading:N.A");
  debug_telemetry_layout->addWidget(_debug_current_heading_label, 1, 0);

  _debug_current_linear_spd_label = new QLabel("Speed:N.A");
  debug_telemetry_layout->addWidget(_debug_current_linear_spd_label, 1, 1);

  _debug_cmd_heading_label = new QLabel("CmdHeading:---");
  debug_telemetry_layout->addWidget(_debug_cmd_heading_label, 2, 0);

  _debug_cmd_turnrate_label = new QLabel("CmdHeadingSec:---");
  debug_telemetry_layout->addWidget(_debug_cmd_turnrate_label, 2, 1);

  _debug_cmd_linear_spd_label = new QLabel("CmdSpeed:---");
  debug_telemetry_layout->addWidget(_debug_cmd_linear_spd_label, 2, 2);

  _debug_telemetry_group_box->setLayout(debug_telemetry_layout);
}

 //--------------------------------------------------------------------------------------------
 // See https://doc.qt.io/qt-5/qtwidgets-layouts-basiclayouts-example.html
 void ImageTrackerPanel::create_commands_group_box()
 {
 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("ImageTrackerPanel::create_commands_group_box() ");
 #endif

   _debug_commands_group_box = new QGroupBox("Commands to AGV State Transition Manager");
   debug_commands_layout = new QGridLayout;

   // 1a) Combo box
   movement_mode_combobox = new QComboBox;
   movement_mode_combobox->setEditable(true);
   movement_mode_combobox->lineEdit()->setPlaceholderText("Select AGV Movement Mode");
   movement_mode_combobox->addItem("WAYPOINT");
   movement_mode_combobox->addItem("MANUAL_STEER");
   debug_commands_layout->addWidget(movement_mode_combobox, 1, 0);

   // 1b) Send button
   movement_mode_pushButton = new QPushButton(tr("&Movement Mode Send"));
   debug_commands_layout->addWidget(movement_mode_pushButton, 1, 1);

   // 1c) Status label
   _debug_movement_mode_label = new QLabel("movement_mode:N.A");
   debug_commands_layout->addWidget(_debug_movement_mode_label, 1, 2);

   // 2) Reset and clear cost map
	reset_clearCostMap_pushButton = new QPushButton(tr("&ResetStates & ClearCostMap"));
	debug_commands_layout->addWidget(reset_clearCostMap_pushButton, 2, 1);

//   // 2a) Combo box
//   fullauto_checkBox = new QCheckBox(tr("&Full Autonomy"));
//   debug_commands_layout->addWidget(fullauto_checkBox, 2, 0);
//
//   // 2b) Send button
//   full_autonomy_pushButton = new QPushButton(tr("&Full Auto Send"));
//   debug_commands_layout->addWidget(full_autonomy_pushButton, 2, 1);

   _debug_commands_group_box->setLayout(debug_commands_layout);
 }

 //--------------------------------------------------------------------------------------------
 // See https://doc.qt.io/qt-5/qtwidgets-layouts-basiclayouts-example.html
 void ImageTrackerPanel::create_engagements_group_box()
 {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::create_engagements_group_box() ");
#endif
	_debug_engagements_group_box = new QGroupBox("AGV Engagements");
	debug_engagements_layout = new QGridLayout;

	// 1a) Combo box
	auto_detector_checkbox = new QCheckBox(tr("&AutoNav (if Trolley Detected)"));
	debug_engagements_layout->addWidget(auto_detector_checkbox, 1, 0);

	// 1b) Send Navigate to Trolley button
	auto_detector_pushButton = new QPushButton(tr("&Navigate to Trolley"));
	auto_detector_pushButton->setEnabled(false);
	debug_engagements_layout->addWidget(auto_detector_pushButton, 1, 1);

	_debug_detector_label = new QLabel("----");
	debug_engagements_layout->addWidget(_debug_detector_label, 1, 2);

	// 2a) Combo box
	auto_docking_checkbox = new QCheckBox(tr("&AutoDock (if Tag Detected)"));
	debug_engagements_layout->addWidget(auto_docking_checkbox, 2, 0);

	// 2b) Send Dock to Tag button
	auto_docking_pushButton = new QPushButton(tr("&Dock to Tag"));
	auto_docking_pushButton->setEnabled(false);
	debug_engagements_layout->addWidget(auto_docking_pushButton, 2, 1);
	_debug_docking_label = new QLabel("----");
	debug_engagements_layout->addWidget(_debug_docking_label, 2, 2);

	// 2b) Send latch commands
	latch_pushButton = new QPushButton(tr("&AGV Latch up"));
	latch_pushButton->setCheckable(true);
	latch_pushButton->setChecked(true);
	latch_pushButton->setStyleSheet("QPushButton{background-color:green;} QPushButton:checked{background-color:red;}");
	debug_engagements_layout->addWidget(latch_pushButton, 3, 1);
	_debug_latch_label = new QLabel("----");
	debug_engagements_layout->addWidget(_debug_latch_label, 3, 2);

	//Nav to [home], [trolley_dropoff], [charge] pushbuttons
	gotoHome_pushButton = new QPushButton(tr("&Goto (home)"));
	gotoHome_pushButton->setEnabled(true);
	debug_engagements_layout->addWidget(gotoHome_pushButton, 4, 0);

	gotoTrolley_dropoff_pushButton = new QPushButton(tr("&Goto Trolley DropOff (tdo)"));
	gotoTrolley_dropoff_pushButton->setEnabled(true);
	debug_engagements_layout->addWidget(gotoTrolley_dropoff_pushButton, 4, 1);

	charge_pushButton = new QPushButton(tr("&Start Charging"));
	charge_pushButton->setCheckable(true);
	charge_pushButton->setChecked(false);
	charge_pushButton->setStyleSheet("QPushButton{background-color:green;} QPushButton:checked{background-color:red;}");

	_debug_voltage_label = new QLabel("--- (low is 24.0v)");	// To show % and cutoff %/voltage
	debug_engagements_layout->addWidget(charge_pushButton, 5, 0);
	debug_engagements_layout->addWidget(_debug_voltage_label, 5, 1);

	_debug_engagements_group_box->setLayout(debug_engagements_layout);

 }

 //--------------------------------------------------------------------------------------------
 void ImageTrackerPanel::create_drive_widget_box()
 {
	 // Original
	 drive_widget_ = new DriveWidget;

	 _debug_drive_widget_box = new QGroupBox("Drive Panel, Pause/Play Button");;
	 //debug_drive_widge_layout = new QVBoxLayout;
	 debug_drive_widge_layout = new QGridLayout;

	 debug_drive_widge_layout->addWidget(drive_widget_, 1, 0);

	 stop_to_stm_pushButton = new QPushButton(tr("Click to Stop AGV"));
	 //pause_play_pushButton->setCheckable(true);
	 //pause_play_pushButton->setChecked(true);
	 //pause_play_pushButton->setStyleSheet("QPushButton{background-color:green;} QPushButton:checked{background-color:red;}");
	 stop_to_stm_pushButton->setStyleSheet("QPushButton{ background-color: yellow }");
	 debug_drive_widge_layout->addWidget(stop_to_stm_pushButton, 1, 1);

	 _debug_drive_widget_box->setLayout(debug_drive_widge_layout);
 }

 //--------------------------------------------------------------------------------------------
 QString ImageTrackerPanel::getImuInTopicString( const QString& fleet_name_topic, const QString& agv_num_topic)
 {
 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("ImageTrackerPanel::getImuInTopicString in(%s : %s) ", fleet_name_topic.toStdString().c_str(), agv_num_topic.toStdString().c_str());
 #endif

 	// Mod by Tim: We are removing the prefix
//     if( fleet_name_topic == "" )
//     {
//       return "";
//     }
//
//     if( agv_num_topic == "" )
//     {
//       return "";
//     }

     // Mod by Tim: We are removing the prefix
// 	QString temp = fleet_name_topic;
// 	QString temp0 = temp.append("_");
// 	QString temp1 = temp.append(agv_num_topic);
// 	QString temp2 = temp1.append("/imu");
// 	QString imu_in_topic = temp2;

     QString imu_in_topic = "/imu";

 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("	getImuInTopicString out(%s) ", imu_in_topic.toStdString().c_str());
 #endif

 	return imu_in_topic;
 }

 //--------------------------------------------------------------------------------------------
 QString ImageTrackerPanel::getOdomInTopicString( const QString& fleet_name_topic, const QString& agv_num_topic)
 {
 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("ImageTrackerPanel::getOdomInTopicString in(%s : %s) ", fleet_name_topic.toStdString().c_str(), agv_num_topic.toStdString().c_str());
 #endif

 	// Mod by Tim: We are removing the prefix
//     if( fleet_name_topic == "" )
//     {
//       return "";
//     }
//
//     if( agv_num_topic == "" )
//     {
//       return "";
//     }

// 	QString temp = fleet_name_topic;
// 	QString temp0 = temp.append("_");
// 	QString temp1 = temp.append(agv_num_topic);
// 	QString temp2 = temp1.append("/odom");
// 	QString imu_in_topic = temp2;
     QString imu_in_topic = "/odom";

 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("	getOdomInTopicString out(%s) ", imu_in_topic.toStdString().c_str());
 #endif

 	return imu_in_topic;
 }

 //--------------------------------------------------------------------------------------------
 QString ImageTrackerPanel::getAmclPoseInTopicString( const QString& fleet_name_topic, const QString& agv_num_topic)
 {
 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("ImageTrackerPanel::getOdomInTopicString in(%s : %s) ", fleet_name_topic.toStdString().c_str(), agv_num_topic.toStdString().c_str());
 #endif

 	// Mod by Tim: We are removing the prefix
//     if( fleet_name_topic == "" )
//     {
//       return "";
//     }
//
//     if( agv_num_topic == "" )
//     {
//       return "";
//     }

     // Mod by Tim: We are removing the prefix
// 	QString temp = fleet_name_topic;
// 	QString temp0 = temp.append("_");
// 	QString temp1 = temp.append(agv_num_topic);
// 	QString temp2 = temp1.append("/amcl_pose");
// 	QString imu_in_topic = temp2;
     QString imu_in_topic = "/amcl_pose";

 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("	getOdomInTopicString out(%s) ", imu_in_topic.toStdString().c_str());
 #endif

 	return imu_in_topic;
 }


 //--------------------------------------------------------------------------------------------
 QString ImageTrackerPanel::getToCmdVelOutTopicString( const QString& fleet_name_topic,  const QString& agv_num_topic)
 {
 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("ImageTrackerPanel::getToCmdVelOutTopicString in(%s : %s) ", fleet_name_topic.toStdString().c_str(), agv_num_topic.toStdString().c_str());
 #endif

 	// Mod by Tim: We are removing the prefix
//     if( fleet_name_topic == "" )
//     {
//       return "";
//     }
//
//     if( agv_num_topic == "" )
//     {
//       return "";
//     }

     // Mod by Tim: We are removing the prefix
// 	QString temp = fleet_name_topic;
// 	QString temp0 = temp.append("_");
// 	QString temp1 = temp.append(agv_num_topic);
// 	QString temp2 = temp1.append("/drive_panel/cmd_vel");
// 	QString cmd_vel_out_topic = temp2;

     QString cmd_vel_out_topic = "/drive_panel/cmd_vel";

 #if DEBUG_IMG_TRK_PANEL
 	ROS_INFO("	getToCmdVelOutTopicString out(%s) ", cmd_vel_out_topic.toStdString().c_str());
 #endif

 	return cmd_vel_out_topic;
 }

//--------------------------------------------------------------------------------------------
 void ImageTrackerPanel::sendStopToSTM(bool stop)
 {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendStopToSTM(%i) ", stop);
#endif

 		// Going to set Red
 	    QString message("Click to Stop AGV");
 	    stop_to_stm_pushButton->setText(message);

 	    atc_msgs::Stop_To_STM msg;
 	    if(stop)
 	    {
 	    	msg.stop = true;
 	    }
 	    else
 	    {
 	    	msg.stop = false;
 	    }
 	    stop_to_stm_publisher_.publish( msg );
 }

 //----------------------------------------------------------------------------------------
 void ImageTrackerPanel::sendLatchCommand(bool latch)
 {
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendLatchCommand(%i) ", latch);
#endif
 		// Going to set Red
 	    atc_msgs::Latch msg;
 	    std::string laserLimitsMinMsg, laserLimitsMaxMsg;

 	    if(latch)
 	    {
 	    	msg.latch = true;
 	    	msg.command_string = "Up.";
 	    	laserLimitsMinMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_min -1.571";
 	    	laserLimitsMaxMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_max 1.571";

 	 	    QString message("AGV Latch down");
 	 	    latch_pushButton->setText(message);
 	    }
 	    else
 	    {
 	    	msg.latch = false;
 	    	msg.command_string = "Down.";
 	    	laserLimitsMinMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_min -2.1";
 	    	laserLimitsMaxMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_max 2.1";

 	    	std_srvs::SetBool latMsg;
 	    	latMsg.request.data = false;
 	    	latch_client.call(latMsg);

  	 	   QString message("AGV Latch Up");
  	 	   latch_pushButton->setText(message);
 	    }
 	   system(laserLimitsMinMsg.c_str());
 	   system(laserLimitsMaxMsg.c_str());
// 	   system(latchStr.c_str());
 	   latch_And_changeLaserLimit_publisher_.publish( msg );

 }

 //----------------------------------------------------------------------------------------
 void ImageTrackerPanel::sendLatchCommand2(bool latch)
 {
	std_srvs::SetBool latMsg;
	std::string laserLimitsMinMsg, laserLimitsMaxMsg;
//	std::string amr_footprint;

//	  max_vel_lin: 0.9
//	  min_vel_lin: 0.6
//	  max_vel_th: 1.0
	std::string eband_max_vel_lin, eband_min_vel_lin, eband_max_vel_th;
	char debug_txt[200];

	if(latch)
	{
		latMsg.request.data = true;
		if(latch_client.call(latMsg))
		{
			ROS_INFO("sendLatchCommand2(latch true) - %s", latMsg.response.message.c_str());

			sprintf(debug_txt,"%s",latMsg.response.message.c_str());
		}
		else
		{
			ROS_WARN("	NOK(latch true) - Failed to call Service [/trolley_lifting_arm_srv]");
			sprintf(debug_txt,"Failed to call Service");
		}

	    laserLimitsMinMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_min -1.571";
	    laserLimitsMaxMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_max 1.571";
//	    amr_footprint = "rosrun dynamic_reconfigure dynparam set [[-1.3,-0.4],[-1.3,0.4],[0.355228,0.4],[0.355228,-0.4]]";
	    eband_max_vel_lin = "rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS  max_vel_lin 0.7";
	    eband_min_vel_lin = "rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS  min_vel_lin 0.4";
	    eband_max_vel_th = "rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS  max_vel_th 0.3";

	 	QString message("AGV Latch2 down");
	 	latch_pushButton->setText(message);
	 	_debug_latch_label->setText(debug_txt);
	}
	else
	{
		latMsg.request.data = false;
		if(latch_client.call(latMsg))
		{
			ROS_INFO("sendLatchCommand2(latch false) - %s", latMsg.response.message.c_str());
		}
		else
		{
			ROS_WARN("	NOK(latch false) - Failed to call Service [/trolley_lifting_arm_srv]");
		}

	    laserLimitsMinMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_min -2.1";
	    laserLimitsMaxMsg = "rosrun dynamic_reconfigure dynparam set /laserscan_multi_merger  angle_max 2.1";
//	    amr_footprint = "rosrun dynamic_reconfigure dynparam set [[-0.166772,-0.27],[-0.166772,0.27],[0.355228,0.27],[0.355228,-0.27]]";
	    eband_max_vel_lin = "rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS  max_vel_lin 0.9";
	    eband_min_vel_lin = "rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS  min_vel_lin 0.6";
	    eband_max_vel_th = "rosrun dynamic_reconfigure dynparam set /move_base/EBandPlannerROS  max_vel_th 1.0";

	 	QString message("AGV Latch2 Up");
	 	latch_pushButton->setText(message);
	 	_debug_latch_label->setText(debug_txt);
	}

	system(laserLimitsMinMsg.c_str());
	system(laserLimitsMaxMsg.c_str());
//	system(amr_footprint.c_str());
	system(eband_max_vel_lin.c_str());
	system(eband_min_vel_lin.c_str());
	system(eband_max_vel_th.c_str());

 }

//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::sendNavigateToTrolley(bool proceed)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendNavigateToTrolley(%i) ", proceed);
#endif

	//update_detector_navigate_srv.request.navigateToTrolley = proceed;
	update_detector_navigate_srv.request.navigateToTrolley = true;

	if (detector_navigate_client.call(update_detector_navigate_srv))
	{
	    ROS_INFO("Ok - M Mode:%i, A_State:%i, [%s], [%s]",
	    		update_detector_navigate_srv.response.movement_mode,
				update_detector_navigate_srv.response.agv_state,
				update_detector_navigate_srv.response.status_message.c_str(),
				update_detector_navigate_srv.response.detector_message.c_str());
	    //printMovementMode(update_movement_mode_srv.response.movement_mode);
	    //printAgvState(update_movement_mode_srv.response.agv_state);

	    char debug_text[150];
	    std::string mode_str = atc_utils::getMovementModeString(update_detector_navigate_srv.response.movement_mode);
	    sprintf (debug_text, "%s mode set", mode_str.c_str());
	    _debug_movement_mode_label->setText(debug_text);

	    std::string state_str = atc_utils::getAgvStateString(update_detector_navigate_srv.response.agv_state);
	    sprintf (debug_text, "%s state", state_str.c_str());
	    _debug_agv_state_label->setText(debug_text);

	    sprintf (debug_text, "%s", update_detector_navigate_srv.response.status_message.c_str());
	    _debug_status_msg_label->setText(debug_text);

	    sprintf (debug_text, "%s", update_detector_navigate_srv.response.detector_message.c_str());
	    _debug_detector_label->setText(debug_text);
	}
	else
	{
	    ROS_WARN("	NOK - Failed to call Service [Navigate To Trolley]");
	}
}

//--------------------------------------------------------------------------------------------
void ImageTrackerPanel::sendDockingToTag(bool proceed)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendDockingToTag(%i) ", proceed);
#endif

	//update_detector_navigate_srv.request.navigateToTrolley = proceed;
	dock_to_tag_srv.request.dock = true;

	if (dock_to_tag_client.call(dock_to_tag_srv))
	{
	    ROS_INFO("	Ok - M Mode:%i, A_State:%i, [%s], [%s]",
	    		dock_to_tag_srv.response.movement_mode,
				dock_to_tag_srv.response.agv_state,
				dock_to_tag_srv.response.status_message.c_str(),
				dock_to_tag_srv.response.dock_message.c_str());
	    //printMovementMode(update_movement_mode_srv.response.movement_mode);
	    //printAgvState(update_movement_mode_srv.response.agv_state);

	    char debug_text[150];
	    std::string mode_str = atc_utils::getMovementModeString(dock_to_tag_srv.response.movement_mode);
	    sprintf (debug_text, "%s mode set", mode_str.c_str());
	    _debug_movement_mode_label->setText(debug_text);

	    std::string state_str = atc_utils::getAgvStateString(dock_to_tag_srv.response.agv_state);
	    sprintf (debug_text, "%s state", state_str.c_str());
	    _debug_agv_state_label->setText(debug_text);

	    sprintf (debug_text, "%s", dock_to_tag_srv.response.status_message.c_str());
	    _debug_status_msg_label->setText(debug_text);

	    sprintf (debug_text, "%s", dock_to_tag_srv.response.dock_message.c_str());
	    _debug_docking_label->setText(debug_text);
	}
	else
	{
	    ROS_WARN("	NOK - Failed to call Service [Dock To Tag]");
	}
}

//----------------------------------------------------------------------------------------
// *New additions (28th July 2021)
void ImageTrackerPanel::sendResetClearCostMap(bool bReset)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendResetClearCostMap(%i) ", bReset);
#endif

	reset_clearCostMap_srv.request.reset_clear_costmap = true;

	if (reset_clear_costmap_client.call(reset_clearCostMap_srv))
	{
	    ROS_INFO("	Ok - M Mode:%i, A_State:%i, [%s]",
	    		reset_clearCostMap_srv.response.movement_mode,
				reset_clearCostMap_srv.response.agv_state,
				reset_clearCostMap_srv.response.status_message.c_str());

	    char debug_text[150];
	    std::string mode_str = atc_utils::getMovementModeString(reset_clearCostMap_srv.response.movement_mode);
	    sprintf (debug_text, "%s mode set", mode_str.c_str());
	    _debug_movement_mode_label->setText(debug_text);

	    std::string state_str = atc_utils::getAgvStateString(reset_clearCostMap_srv.response.agv_state);
	    sprintf (debug_text, "%s state", state_str.c_str());
	    _debug_agv_state_label->setText(debug_text);

	    sprintf (debug_text, "%s", reset_clearCostMap_srv.response.status_message.c_str());
	    _debug_status_msg_label->setText(debug_text);
	}
	else
	{
	    ROS_WARN("	NOK - Failed to call Service [Reset_ClearCostMap]");
	}
}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::sendGotoHome(bool bHome)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendGotoHome(%i) ", bHome);
#endif
	atc_msgs::RunSpecificWp msg;
	msg.name = "home";
	run_specific_wp_publisher_.publish( msg );
}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::sendGotoTrolleyDropOff(bool bTrolleyDropOff)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendGotoTrolleyDropOff(%i) ", bTrolleyDropOff);
#endif
	atc_msgs::RunSpecificWp msg;
	msg.name = "tdo";
	run_specific_wp_publisher_.publish( msg );

}

//----------------------------------------------------------------------------------------
void ImageTrackerPanel::sendChargeAGV(bool bCharge)
{
#if DEBUG_IMG_TRK_PANEL
	ROS_INFO("ImageTrackerPanel::sendChargeAGV(%i) ", bCharge);
#endif

	// Going to set Red
	std_srvs::Empty msg;
	if(bCharge)
	{
		charge_agv_client.call(msg);

	 	QString message("Stop Charging");
		charge_pushButton->setText(message);
	}
	else
	{
		stop_charge_agv_client.call(msg);

	 	QString message("Start Charging");
	 	charge_pushButton->setText(message);
	}

}



} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ImageTrackerPanel,rviz::Panel )
// END_TUTORIAL
