#
#   robotiq_s_model_control_client.py
#
#   Created on: May 23, 2014
#   Authors:   Francisco Vina
#             fevb <at> kth.se 
#

#  Copyright (c) 2014, Francisco Vina, CVAP, KTH
#    All rights reserved.

#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#       * Redistributions of source code must retain the above copyright
#         notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#       * Neither the name of KTH nor the
#         names of its contributors may be used to endorse or promote products
#         derived from this software without specific prior written permission.

#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
#    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#

import rospy
from robotiq_s_model_control.msg import *



class RobotiqSModelControlClient():
    """
    Control client for sending commands to Robotiq S-model hand
    All parameters are 8 bit integers between 0 and 255
    """

    def __init__(self):
        self.GM_basic = 0
        self.GM_pinch = 1
        self.GM_scissor = 2
        self.GM_wide = 3

        self.grasping_modes = [self.GM_basic, self.GM_pinch, self.GM_scissor, self.GM_wide]

        self.publisher = rospy.Publisher('/SModelRobotOutput',
                                         SModel_robot_output)

        self.subscriber = rospy.Subscriber('/SModelRobotInput',
                                           SModel_robot_input,
                                           self.s_model_robot_input_callback)

        self._current_grasping_mode = self.GM_basic

        self._received_status_msg = False


    def activate(self):
        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 150
        command.rFRA = 150
        self.publisher.publish(command)

    def reset(self):
        command = SModel_robot_output()
        command.rACT = 0
        self.publisher.publish(command)

    def set_grasping_mode(self, grasping_mode):
        self._current_grasping_mode = grasping_mode

    def open(self, speed=150, force=150):
        speed_ = self._saturate_param(speed)
        force_ = self._saturate_param(force)

        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rPRA = 0
        command.rSPA = speed_
        command.rFRA = force_

        self.publisher.publish(command)

    def close(self, speed=150, force=150):
        speed_ = self._saturate_param(speed)
        force_ = self._saturate_param(force)

        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rPRA = 255
        command.rSPA = speed_
        command.rFRA = force_

        self.publisher.publish(command)


    def goto_pos(self, pos, speed, force):
        """
        parameters are of list type
        sends hand to pos if parameters are of size 1
        OR
        sends individual position commands to each finger if
        the parameters are of size 3
        """
        pos_ = self._saturate_param(pos)
        speed_ = self._saturate_param(speed)
        force_ = self._saturate_param(force)

        # simple control mode
        if len(pos)==1:
            if len(speed)!=1 or len(force)!=1:
                rospy.logerr('Speed or force parameter size != 1 for simple goto_pos')
                return

            command = SModel_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rMOD = self._current_grasping_mode

            command.rICF = 0
            command.rICS = 0

            command.rPRA = pos_[0]
            command.rSPA = speed_[0]
            command.rFRA = force_[0]

            self.publisher.publish(command)

        # advanced (individual) control mode
        else:
            if len(pos)!=3 or len(speed)!=3 or len(force)!=3:
                rospy.logerr('Invalid param size for goto_pos command')

            command = SModel_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rMOD = self._current_grasping_mode

            command.rICF = 1
            command.rICS = 0

            command.rPRA = pos_[0]
            command.rSPA = speed_[0]
            command.rFRA = force_[0]

            command.rPRB = pos_[1]
            command.rSPB = speed_[1]
            command.rFRB = force_[1]

            command.rPRC = pos_[2]
            command.rSPC = speed_[2]
            command.rFRC = force_[2]
            self.publisher.publish(command)



    def goto_scissor_pos(self, pos, speed, force):
        """
        Control the "scissor" axis
        Parameters are 1 dimensional unsigned int 0-255
        """
        pos_ = self._saturate_param(pos)
        speed_ = self._saturate_param(speed)
        force_ = self._saturate_param(force)


        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rICF = 0
        command.rICS = 1

        command.rPRS = pos_
        command.rSPS = speed_
        command.rFRS = force_

        self.publisher.publish(command)

    def goto_configuration(self, config, speed, force):
        """
        Move the hand to a 4 DOF configuration (position). 
        Each value is expected to be in a range of 0 to 255.
        :param config (j0, j1, j2, j3) where j0 is the scissor joint, j1 finger 1, j2 finger 2 and j3 finger 3.
        :param speed (v0, v1, v2, v3) speeds for the respective joints
        :param force (f0, f1, f2, f3) forces for the respective joints
        """
        config = self._saturate_param(config)
        speed = self._saturate_param(speed)
        force = self._saturate_param(force)

        command = SModel_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rICF = 1
        command.rICS = 1
        command.rPRS = config[0]
        command.rSPS = speed[0]
        command.rFRS = force[0]
        command.rPRA = config[1]
        command.rSPA = speed[1]
        command.rFRA = force[1]
        command.rPRB = config[2]
        command.rSPB = speed[2]
        command.rFRB = force[2]
        command.rPRC = config[3]
        command.rSPC = speed[3]
        command.rFRC = force[3]
        self.publisher.publish(command)

    def s_model_robot_input_callback(self, msg):
        """
        callback function for receiving status updates of the
        Robotiq hand
        """
        self._status_msg = msg

    def get_grasping_mode(self):
        return self._status_msg.gMOD


    def in_reset_mode(self):
        return self._status_msg.gIMC==0

    def activation_ongoing(self):
        return self._status_msg.gIMC==1

    def mode_change_ongoing(self):
        return self._status_msg.gIMC==2

    def activation_completed(self):
        return self._status_msg.gIMC==3

    def mode_change_completed(self):
        return self._status_msg.gIMC==3

    def in_motion(self):
        return self._status_msg.gSTA==0

    def stopped_partially_before_requested_position(self):
        return self._status_msg.gSTA==1

    def stopped_full_before_requested_position(self):
        return self._status_msg.gSTA==2

    def stopped_at_requested_position(self):
        return self._status_msg.gSTA==3


    def _saturate_param(self, param):
        """
        Saturate parameters between 0 and 255
        """
        param_ = param
        if(type(param)==int):
            if(param_<0):
                param_ = 0

            if(param_>255):
                param_ = 255

        else:
            for x in xrange(0,len(param)):
                if(param_[x] < 0):
                    param_[x] = 0

                if(param_[x] > 255):
                    param_[x] = 255

        return param_