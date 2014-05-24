/*
 * RobotiqSModelControlClient.cpp
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

#include <robotiq_s_model_control_client/RobotiqSModelControlClient.h>
#include <robotiq_s_model_control/SModel_robot_output.h>

using namespace robotiq_s_model_control;


RobotiqSModelControlClient::RobotiqSModelControlClient()
{
	n_ = ros::NodeHandle();
	topicPub_SModelRobotOutput_ = n_.advertise<SModel_robot_output>("/SModelRobotOutput", 1);
	topicSub_SModelRobotInput_ = n_.subscribe("/SModelRobotInput", 1,
			&RobotiqSModelControlClient::topicCallback_SModelRobotInput, this);

	m_current_grasping_mode = GM_basic;
	m_received_status_msg = false;

}

RobotiqSModelControlClient::~RobotiqSModelControlClient()
{
}

void RobotiqSModelControlClient::activate()
{
	SModel_robot_output command = SModel_robot_output();
	command.rACT = 1;
	command.rGTO = 1;
	command.rSPA = 150;
	command.rFRA = 150;
	topicPub_SModelRobotOutput_.publish(command);
}

void RobotiqSModelControlClient::reset()
{
	SModel_robot_output command = SModel_robot_output();
	command.rACT = 0;
	topicPub_SModelRobotOutput_.publish(command);
}


void RobotiqSModelControlClient::setGraspingMode(const grasping_mode& mode)
{
	m_current_grasping_mode = mode;
}


bool RobotiqSModelControlClient::open(unsigned int speed, unsigned int force)
{

	unsigned int speed_ = speed;
	unsigned int force_ = force;

	// saturate the input params

	if(speed_>255) speed_ = 255;
	if(force_>255) force_ = 255;


	SModel_robot_output command = SModel_robot_output();
	command.rACT = 1;
	command.rGTO = 1;
	command.rPRA = (uint8_t) 0;
	command.rSPA = (uint8_t) speed_;
	command.rFRA = (uint8_t) force_;
	topicPub_SModelRobotOutput_.publish(command);

	return true;
}


bool RobotiqSModelControlClient::close(unsigned int speed, unsigned int force)
{
	unsigned int speed_ = speed;
	unsigned int force_ = force;

	if(speed_>255) speed_ = 255;
	if(force_>255) force_ = 255;

	SModel_robot_output command = SModel_robot_output();
	command.rACT = 1;
	command.rGTO = 1;
	command.rPRA = (uint8_t) 255;
	command.rSPA = (uint8_t) speed_;
	command.rFRA = (uint8_t) force_;
	topicPub_SModelRobotOutput_.publish(command);


	return true;
}


bool RobotiqSModelControlClient::gotoPos(const std::vector<unsigned int> &pos,
		const std::vector<unsigned int> &speed,
		const std::vector<unsigned int> &force)
{

	std::vector<unsigned int> pos_ = pos;
	std::vector<unsigned int> speed_ = speed;
	std::vector<unsigned int> force_ = force;

	// first saturate the params to range 0-255

	for(unsigned int i =0; i<pos_.size(); i++)
	{
		if(pos_[i]>255) pos_[i] = 255;
	}


	for(unsigned int i =0; i<speed_.size(); i++)
	{
		if(speed_[i]>255) speed_[i] = 255;
	}


	for(unsigned int i =0; i<force_.size(); i++)
	{
		if(force_[i]>255) force_[i] = 255;
	}


	// simple control mode
	if(pos_.size()==1)
	{

		if(pos_.size()!=1 || speed_.size()!=1 || force_.size()!=1)
		{
			ROS_ERROR("RobotiqSModelControlClient: pos/speed/force vector size != 1 in simple control mode");
			return false;
		}

		SModel_robot_output command = SModel_robot_output();
		command.rACT = 1;
		command.rGTO = 1;
		command.rMOD = m_current_grasping_mode;

		command.rICF = 0;
		command.rICS = 0;

		command.rPRA = (uint8_t) pos_[0];
		command.rSPA = (uint8_t) speed_[0];
		command.rFRA = (uint8_t) force_[0];

		topicPub_SModelRobotOutput_.publish(command);
	}

	// advanced (individual finger) control mode
	else
	{
		if(pos_.size()!=3 || speed_.size()!=3 || force_.size()!=3)
		{
			ROS_ERROR("RobotiqSModelControlClient: pos/speed/force vector size != 3 in advanced control mode");
			return false;
		}

		SModel_robot_output command = SModel_robot_output();
		command.rACT = 1;
		command.rGTO = 1;
		command.rMOD = m_current_grasping_mode;

		command.rICF = 1;
		command.rICS = 0;

		command.rPRA = (uint8_t) pos_[0];
		command.rSPA = (uint8_t) speed_[0];
		command.rFRA = (uint8_t) force_[0];

		command.rPRB = (uint8_t) pos_[1];
		command.rSPB = (uint8_t) speed_[1];
		command.rFRB = (uint8_t) force_[1];

		command.rPRC = (uint8_t) pos_[2];
		command.rSPC = (uint8_t) speed_[2];
		command.rFRC = (uint8_t) force_[2];
		topicPub_SModelRobotOutput_.publish(command);

	}


	return true;
}

void RobotiqSModelControlClient::gotoScissorPos(unsigned int pos, unsigned int speed, unsigned int force)
{

	unsigned int pos_ = pos;
	unsigned int speed_ = speed;
	unsigned int force_ = force;

	// first saturate the input params to range 0-255
	if(pos_>255) pos_ = 255;
	if(speed_>255) speed_ = 255;
	if(force_>255) force_ = 255;

	SModel_robot_output command = SModel_robot_output();
	command.rACT = 1;
	command.rGTO = 1;
	command.rICF = 0;
	command.rICS = 1;

	command.rPRS = (uint8_t) pos_;
	command.rSPS = (uint8_t) speed_;
	command.rFRS = (uint8_t) force_;

	topicPub_SModelRobotOutput_.publish(command);

}

void RobotiqSModelControlClient::topicCallback_SModelRobotInput(
		const SModel_robot_inputPtr& msg)
{

	m_status_msg = *msg;
}

SModel_robot_input RobotiqSModelControlClient::getStatusMsg()
{
	return m_status_msg;
}

RobotiqSModelControlClient::grasping_mode RobotiqSModelControlClient::getGraspingMode()
{
	switch (m_status_msg.gMOD)
	{
	case 0:
		return GM_basic;
	case 1:
		return GM_pinch;
	case 2:
		return GM_scissor;
	case 3:
		return GM_wide;
	}

}

bool RobotiqSModelControlClient::inResetMode()
{
	return m_status_msg.gIMC==0;
}

bool RobotiqSModelControlClient::activationOngoing()
{
	return m_status_msg.gIMC==1;
}

bool RobotiqSModelControlClient::modeChangeOngoing()
{
	return m_status_msg.gIMC==2;
}

bool RobotiqSModelControlClient::activationCompleted()
{
	return m_status_msg.gIMC==3;
}

bool RobotiqSModelControlClient::modeChangeCompleted()
{
	return m_status_msg.gIMC==3;
}

bool RobotiqSModelControlClient::inMotion()
{
	return m_status_msg.gSTA==0;
}

bool RobotiqSModelControlClient::stoppedPartiallyBeforeRequestedPosition()
{
	return m_status_msg.gSTA==1;
}

bool RobotiqSModelControlClient::stoppedFullBeforeRequestedPosition()
{
	return m_status_msg.gSTA==2;
}

bool RobotiqSModelControlClient::stoppedAtRequestedPosition()
{
	return m_status_msg.gSTA==3;
}


