
/**
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <signal.h>
#include <stdio.h>

/* --Includes-- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "astm_enums.h"

#include "atc_utils/atc_utils.h"

#include "atc_msgs/Update_Movement_Mode.h"
#include "atc_msgs/PE_Has_Solution.h"
#include "atc_msgs/AprilTag_Has_Solution.h"
#include "atc_msgs/Click_To_Turn.h"

void sigint(int a)
{
   printf("Caught signal %i, exiting... \n", a);
   exit(1);
}

int main(int argc, char **argv)
{
	signal(SIGINT, sigint);

  ros::init(argc, argv, "test_stub", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  ros::ServiceClient movement_mode_client = n.serviceClient<atc_msgs::Update_Movement_Mode>("atc_stm/Update_Movement_Mode");
  ros::ServiceClient pe_client = n.serviceClient<atc_msgs::PE_Has_Solution>("atc_stm/PE_Has_Solution");
  ros::ServiceClient at_client = n.serviceClient<atc_msgs::AprilTag_Has_Solution>("atc_stm/AprilTag_Has_Solution");

  atc_msgs::Update_Movement_Mode update_movement_mode_srv;
  atc_msgs::PE_Has_Solution pe_srv;
  atc_msgs::AprilTag_Has_Solution at_srv;

  // Load parameter
//  if (n.hasParam("theta"))
//  {
//	bool getTheta =  n.getParam("theta", pe_srv.request.theta);
//    if (!getTheta)
//    {
//      ROS_ERROR("Error Getting theta \n");
//      return -1;
//    }
//    else
//    {
//    	ROS_INFO("Parameter theta:%.2f available", pe_srv.request.theta);
//    }
//  }

  while(1)
  {
	  int c;
	  int movement_mode;
	  int PE_has_solution;
	  int AT_has_solution;

	  printf("1) Key 0-3 for movement mode (see astm_enums.h) \n");
	  printf("	0 for UNKNOWN_MODE \n");
	  printf("	1 for WAYPOINT \n");
	  printf("	2 for MANUAL_STEER \n");
	  printf("	3 for STOP \n");
	  while ((c = getchar()) != '\n' && c != EOF)
	  {
	    movement_mode = c - '0';
	  }
	  printf("movement_mode:%i \n\n", movement_mode);

	  printf("2) Key 0-1 for Pose Estimator has solution \n");
	  while ((c = getchar()) != '\n' && c != EOF)
	  {
	    PE_has_solution = c - '0';
	  }
	  printf("PE_has_solution:%i \n\n", PE_has_solution);

	  printf("3) Key 0-1 for AprilTag has solution \n");
	  while ((c = getchar()) != '\n' && c != EOF)
	  {
	    AT_has_solution = c - '0';
	  }
	  printf("movement_mode:%i PE_has_solution:%i AT_has_solution:%i \n\n", movement_mode, PE_has_solution, AT_has_solution);



	  //-----------------------------------------------------------------------------------------------
	  // 1) Update Movememnt Mode
	  //atc_state_machine_manager::Update_Movement_Mode update_movement_mode_srv;
	  //update_movement_mode_srv.request.movement_mode = astm::MOVEMENT_MODE::MANUAL_STEER;
	  update_movement_mode_srv.request.movement_mode = movement_mode;

	  ROS_INFO("4) Updating Movement Mode...");
	  if (movement_mode_client.call(update_movement_mode_srv))
	  {
	    ROS_INFO("	Ok - M Mode:%i, A_State:%i, Status_msg:[%s]",
	    		update_movement_mode_srv.response.movement_mode,
				update_movement_mode_srv.response.agv_state,
				update_movement_mode_srv.response.status_message.c_str());
	    atc_utils::printMovementMode(update_movement_mode_srv.response.movement_mode);
	    atc_utils::printAgvState(update_movement_mode_srv.response.agv_state);
	  }
	  else
	  {
	    ROS_WARN("	NOK - Failed to call Service [Update_Movement_Mode]");
	  }

	  //-----------------------------------------------------------------------------------------------
	  // 2) Mimic Pose Estimator has solution
	  //atc_state_machine_manager::PE_Has_Solution pe_srv;

	  // Mod by Tim:
	  //pe_srv.request.hasSolution = (PE_has_solution > 0) ? (true) : (false);
//	  pe_srv.request.x = 0.1;
//	  pe_srv.request.y = 0.2;
//	  pe_srv.request.theta = 90.0;
	  pe_srv.request.boxes[0].pixelPosRight = 0.5;
	  pe_srv.request.boxes[0].pixelPosDown = 0.5;
	  pe_srv.request.boxes[0].width = 1.0;
	  pe_srv.request.boxes[0].height = 1.0;

	  //  This actually calls the service
	  ROS_INFO("5) Updating Coarse Guidance...");
	  if (pe_client.call(pe_srv))
	  {
	    ROS_INFO("	Ok - M Mode:%i, A_State:%i, Status_msg:[%s]",
	    		pe_srv.response.movement_mode,
				pe_srv.response.agv_state,
				pe_srv.response.status_message.c_str());
	    atc_utils::printMovementMode(pe_srv.response.movement_mode);
	    atc_utils::printAgvState(pe_srv.response.agv_state);
	  }
	  else
	  {
	    ROS_WARN("	NOK - Failed to call Service [PE_Has_Solution]");
	  }
	  //-----------------------------------------------------------------------------------------------
	  // 3) Mimic AprilTag has solution
	  // Mod by Tim:
	  at_srv.request.hasSolution = (AT_has_solution > 0) ? (true) : (false);
	  at_srv.request.x = 0.3;
	  at_srv.request.y = 0.4;
	  at_srv.request.theta = 100.0;

	  //  This actually calls the service
	  ROS_INFO("5) Updating Terminal Guidance...");
	  if (at_client.call(at_srv))
	  {
	    ROS_INFO("	Ok - M Mode:%i, A_State:%i, Status_msg:[%s]",
	    		at_srv.response.movement_mode,
				at_srv.response.agv_state,
				at_srv.response.status_message.c_str());
	    atc_utils::printMovementMode(at_srv.response.movement_mode);
	    atc_utils::printAgvState(at_srv.response.agv_state);
	  }
	  else
	  {
	    ROS_WARN("	NOK - Failed to call Service [AprilTag_Has_Solution]");
	  }



  }



  return 0;
}
