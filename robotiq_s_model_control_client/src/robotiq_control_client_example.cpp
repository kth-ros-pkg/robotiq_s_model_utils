/*
 *  robotiq_control_client_example.cpp
 *
 *  Created on: Aug 6, 2013
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <robotiq_s_model_control_client/RobotiqSModelControlClient.h>


int main(int argc, char **argv) {

	ros::init(argc, argv, "robotiq_hand_control_client_test");

	RobotiqSModelControlClient robotiq_control_client;

	// first activate the hand
	ROS_INFO("Activating the hand");
	robotiq_control_client.activate();
	ros::Duration(5.0).sleep();

	// set basic grasping mode
	ROS_INFO("Setting basic grasp mode");
	robotiq_control_client.setGraspingMode(RobotiqSModelControlClient::GM_basic);


	ROS_INFO("Closing the hand");
	robotiq_control_client.close();
	ros::Duration(3.0).sleep();


	// do individual finger control
	std::vector<unsigned int> pos(3, 255);
	std::vector<unsigned int> speed(3, 255);
	std::vector<unsigned int> force(3, 120);

	pos[0] = 80;
	ROS_INFO("Controlling position of Finger A");
	robotiq_control_client.gotoPos(pos, speed, force);
	ros::Duration(5.0).sleep();


	ROS_INFO("Opening the hand");
	robotiq_control_client.open();
	ros::Duration(6.0).sleep();
	return 0;
}


