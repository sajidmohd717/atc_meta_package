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
#ifndef IMAGE_TRACKER_PANEL_H
#define IMAGE_TRACKER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

// Mod by Tim:
#include <QLabel>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QComboBox>
#include <QTextEdit>
#include <QCheckBox>
#include <QPushButton>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "ImageTrackerInfo.h"

// ROS ATC State Transition Manager Services
#include "atc_msgs/Update_Movement_Mode.h"
#include "atc_msgs/Click_To_Turn.h"
#include "atc_msgs/Navigate_To_Trolley.h"
#include "atc_msgs/Dock_To_Tag.h"

// ROS ATC State Transition Manager Messages
#include "atc_msgs/Detector_Has_Solution.h"	// Incoming
#include "atc_msgs/AprilTag_Has_Solution.h"	// Incoming
#include "atc_msgs/AGVStatus.h"		// Incoming
#include "atc_msgs/Stop_To_STM.h"	// Outgoing
#include "atc_msgs/Latch.h"	// Outgoing

// *New additions (28th July 2021), Outgoing
#include "atc_msgs/Reset_ClearCostMap.h"
#include "atc_msgs/RunSpecificWp.h"

// For Xnergy charging and status msgs
#include <plc_modbus_node/xnergy_sensors.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

class QLineEdit;

namespace rviz_plugin_tutorials
{

class DriveWidget;


class ImageTrackerPanel: public rviz::Panel
{
Q_OBJECT
public:
  ImageTrackerPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void setHeadingCmd( const float& heading_deg_, const float& ang_velocity_radsec);
  void setCmdVelOutTopic(  QString& topic );
  void setImuOdomAmclPoseInTopic(  QString& imuTopic, QString& odomTopic, QString& amclPoseTopic );

protected Q_SLOTS:
  void sendDriveWidgetCmd();
  void subscribeAgvTopics();
  void updateFleetName();
  void updateAgvNumber(QString new_number_string);

  void updateMovementMode(int index);
  void sendMovementMode();

  void sendCTTHeadingHold(const double& cmd_heading);

  void sendNavigateToTrolley(bool proceed);
  void sendDockingToTag(bool proceed);
  void sendStopToSTM(bool stop);
  void sendLatchCommand(bool latch);
  void sendLatchCommand2(bool latch);

  // *New additions (28th July 2021)
  void sendResetClearCostMap(bool bReset);
  void sendGotoHome(bool bHome);
  void sendGotoTrolleyDropOff(bool bTrolleyDropOff);
  void sendChargeAGV(bool bCharge);

protected:
  ros::NodeHandle nh_;
  ros::Publisher velocity_publisher_;
  ros::Publisher stop_to_stm_publisher_;
  ros::Publisher latch_And_changeLaserLimit_publisher_;
  ros::Publisher run_specific_wp_publisher_;

  // Messages
  ros::Subscriber agv_state_subscriber;
  ros::Subscriber agv_odom_subscriber;
  ros::Subscriber agv_amcl_pose_subscriber;
  ros::Subscriber agv_detector_pb_subscriber;	// msg to enable pushbutton
  ros::Subscriber agv_docking_pb_subscriber;	// msg to enable pushbutton
  ros::Subscriber xnergy_subscriber;
  ros::Subscriber cmd_vel_subscriber;
  void agvStatusCallback(const atc_msgs::AGVStatus::ConstPtr& agv_status_msg);

  // Services
  ros::ServiceClient movement_mode_client;
  atc_msgs::Update_Movement_Mode update_movement_mode_srv;
  ros::ServiceClient ctt_client;
  atc_msgs::Click_To_Turn update_ctt_srv;
  ros::ServiceClient detector_navigate_client;
  atc_msgs::Navigate_To_Trolley update_detector_navigate_srv;
  ros::ServiceClient dock_to_tag_client;
  atc_msgs::Dock_To_Tag dock_to_tag_srv;

  // *New additions (28th July 2021)
  ros::ServiceClient reset_clear_costmap_client;
  atc_msgs::Reset_ClearCostMap reset_clearCostMap_srv;
//  ros::ServiceClient goto_home_client;
//  atc_msgs::Goto_Home goto_home_srv;
//  ros::ServiceClient goto_trolley_dropoff_client;
//  atc_msgs::Goto_Trolley_DropOff goto_trolley_dropoff_srv;
  ros::ServiceClient charge_agv_client;
  ros::ServiceClient stop_charge_agv_client;
  ros::ServiceClient latch_client;

  float angular_velocity_radsec;
  float cmd_odom_heading_deg_;
  float odom_heading_degs;

//  std::mutex _imu_mutex;
//  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_msg);
  void detectorPBCallback(const atc_msgs::Detector_Has_Solution::ConstPtr& detect_pb_msg);
  void dockingPBCallback(const atc_msgs::AprilTag_Has_Solution::ConstPtr& docking_pb_msg);
  void xnergyCallback(const plc_modbus_node::xnergy_sensors::ConstPtr& xnergy_msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);

  // ----------- Topic labels -----------
  QGroupBox* _debug_topic_group_box;
  //QHBoxLayout* debug_topic_layout;
  QGridLayout* debug_topic_layout;
  //QLineEdit* output_topic_editor_;	// cmd_vel
  //QString output_topic_;
  QString steer_cmds_topic_;

  //QLineEdit* input_imu_topic_editor_;	// imu
  //QString imuIn_topic_;
  QLineEdit* fleet_name_editor_;
  QString fleet_name_;
  QString agv_number_;
  QString imuIn_topic_;
  QString odomIn_topic_;
  QString cmdVelOut_topic_;
  QString amclPoseIn_topic_;
  QPushButton *subscribe_pushButton;
  QComboBox* agv_number_combobox;
  QLabel* _debug_fleet_agv_connected_to;

  // ----------- Status labels -----------
  QGroupBox* _debug_status_group_box;
  QGridLayout* debug_status_layout;
  QLabel* _debug_status_msg_label;
  QLabel* _debug_agv_state_label;


  // ----------- Telemetry labels -----------
  QGroupBox* _debug_telemetry_group_box;
  QGridLayout* debug_telemetry_layout;
  QLabel* _debug_current_heading_label;
  QLabel* _debug_cmd_heading_label;
  QLabel* _debug_cmd_turnrate_label;
  QLabel* _debug_current_linear_spd_label;
  QLabel* _debug_cmd_linear_spd_label;
  double east_meters;
  double north_meters;
  double linear_speed_ms;

  // ----------- Commands to AGV -----------
  QGroupBox* _debug_commands_group_box;
  QGridLayout* debug_commands_layout;
  QComboBox* movement_mode_combobox;
  QLabel* _debug_movement_mode_label;
  QPushButton *movement_mode_pushButton;
  QPushButton *reset_clearCostMap_pushButton;
  int movement_mode;

  // ----------- Engagements to AGV -----------
  QGroupBox* _debug_engagements_group_box;
  QGridLayout* debug_engagements_layout;
  // Detector (Trolley) Goal - Map to show trolley icon
  QCheckBox* auto_detector_checkbox;
  QPushButton *auto_detector_pushButton;
  QLabel* _debug_detector_label;
  // Apriltag Docking
  QCheckBox* auto_docking_checkbox;
  QPushButton *auto_docking_pushButton;
  QLabel* _debug_docking_label;
  // Latch / Unlatch button
  QPushButton *latch_pushButton;
  QLabel* _debug_latch_label;
  // Nav to [home], [trolley_dropoff], [charge] pushbuttons
  QPushButton *gotoHome_pushButton;
  QPushButton *gotoTrolley_dropoff_pushButton;
  QPushButton *charge_pushButton;
  QLabel* _debug_voltage_label;

  // TODO: Other pushbuttons to add:
  // RTB (return to base) Pushbutton
  // Charge battery Pushbutton (Enabled after dock; cmds to Xnergy)
  // Reset ATC_STM
  // Config settings (to STM) button

  // ----------- Drive Widget Box-----------
  DriveWidget* drive_widget_;
  QGroupBox* _debug_drive_widget_box;
  //QVBoxLayout* debug_drive_widge_layout;
  QGridLayout* debug_drive_widge_layout;
  QPushButton* stop_to_stm_pushButton;



private:
  void create_topic_box();
  void create_commands_group_box();
  void create_status_group_box();
  void create_telemetry_group_box();
  void create_drive_widget_box();
  void create_engagements_group_box();

  QString getImuInTopicString( const QString& fleet_name_topic,  const QString& agv_num_topic);
  QString getOdomInTopicString( const QString& fleet_name_topic, const QString& agv_num_topic);
  QString getAmclPoseInTopicString( const QString& fleet_name_topic, const QString& agv_num_topic);
  QString getToCmdVelOutTopicString( const QString& fleet_name_topic,  const QString& agv_num_topic);

};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
