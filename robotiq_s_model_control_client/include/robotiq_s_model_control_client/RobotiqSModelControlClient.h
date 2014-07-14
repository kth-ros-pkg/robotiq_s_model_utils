/*
 * RobotiqSModelControlClient.h
 *
 *  Created on: Aug 4, 2013
 *      Author: fevb
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

#ifndef ROBOTIQSMODELCONTROLCLIENT_H_
#define ROBOTIQSMODELCONTROLCLIENT_H_
#include <ros/ros.h>
#include <robotiq_s_model_control/SModel_robot_input.h>


//  NB: in order to get the status messages from the server, SpinOnce() must be called from the main code !!!!

using namespace robotiq_s_model_control;

class RobotiqSModelControlClient
{

public:

	enum grasping_mode
	{
		GM_basic,
		GM_pinch,
		GM_wide,
		GM_scissor
	};

	ros::NodeHandle n_;
	ros::Publisher topicPub_SModelRobotOutput_;
	ros::Subscriber topicSub_SModelRobotInput_;

	// default speed and force are set to 150
	RobotiqSModelControlClient();
	virtual ~RobotiqSModelControlClient();


	// --------------------- Function calls for sending commands to the Robotiq hand ---------------------


	// must be called first
	void activate();

	void reset();


	// grasp using preshapes
	void setGraspingMode(const grasping_mode &mode);



	// opens the hand (fingers A-C). Only for simple control mode
	bool open(unsigned int speed=150, unsigned int force=150);

	// closes the hand (fingers A-C). Only for simple control mode
	bool close(unsigned int speed=150, unsigned int force=150);


	// executes the position request command for fingers A,B,C
	// pos/speed/force should be size 1 for simple control mode
	// otherwise they should be size 3 for advanced control mode (control individual fingers)
	// pos/speed/force[0] --> finger A
	// pos/speed/force[1] --> finger B
	// pos/speed/force[2] --> finger C
	bool gotoPos(const std::vector<unsigned int> &pos,
			const std::vector<unsigned int> &speed,
			const std::vector<unsigned int> &force);

	// sets position for scissor axis
	void gotoScissorPos(unsigned int pos, unsigned int speed = 150, unsigned int force = 150);


	// --------------------- Function calls for reading status of the Robotiq hand ---------------------
	// NB: Need to ros::SpinOnce() in the main code in order to receive the messages!!!

	void topicCallback_SModelRobotInput(const SModel_robot_inputPtr &msg);

	SModel_robot_input getStatusMsg();

	grasping_mode getGraspingMode();

	// bool Stopped();

	bool inResetMode();

	bool activationOngoing();

	bool modeChangeOngoing();

	bool activationCompleted();

	bool modeChangeCompleted();

	bool inMotion();

	bool stoppedPartiallyBeforeRequestedPosition();

	bool stoppedFullBeforeRequestedPosition();

	bool stoppedAtRequestedPosition();



private:

	grasping_mode m_current_grasping_mode;
	SModel_robot_input m_status_msg;
	bool m_received_status_msg;


};

#endif
