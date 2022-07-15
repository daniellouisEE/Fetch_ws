#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import argparse
import subprocess
import sys
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface

class MoveItThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.start()

    def run(self):
        self.process = subprocess.Popen(["roslaunch", "fetch_moveit_config", "move_group.launch", "--wait"])
        _, _ = self.process.communicate()

    def stop(self):
        self.process.send_signal(subprocess.signal.SIGINT)
        self.process.wait()

def is_moveit_running():
    output = subprocess.check_output(["rosnode", "info", "move_group"], stderr=subprocess.STDOUT)
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

class TuckThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = None
        self.start()

    def run(self):
        move_thread = None
        if not is_moveit_running():
            rospy.loginfo("starting moveit")
            move_thread = MoveItThread()

        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")
        
        base = PlanningSceneInterface("base_link")
        torso = PlanningSceneInterface("torso_lift_link")
        head = PlanningSceneInterface("torso_lift_link")
        face = PlanningSceneInterface("head_tilt_link")
        #ground protection
        base.addCube("my_front_ground", 2, 1.1, 0.0, -0.97)
        base.addCube("my_back_ground", 2, -1.2, 0.0, -0.97)
        base.addCube("my_left_ground", 2, 0.0, 1.2, -0.97)
        base.addCube("my_right_ground", 2, 0.0, -1.2, -0.97)
        face.addBox("face", 0.0, 0.25, 0.17, 0.11, 0.0, 0.0)
        #link face to torso instead of headtilt because we need the box to go up and down with the torso
        #robot body protection  
        base.addBox("base", 0.3, 0.55, 0.05, 0.15, 0.0, 0.36)
        base.addBox("base_right",0.3, 0.05, 0.35, 0.15, -0.25, 0.23)
        base.addBox("base_left", 0.3, 0.05, 0.35, 0.15, 0.25, 0.23)
        base.addBox("base_front", 0.05, 0.46, 0.35, 0.35, 0.0, 0.23)
        head.addBox("head", 0.3, 0.3, 0.0, 0.1, 0.0, 0.8)
        torso.addBox("torso_left", 0.2, 0.0, 1.0, 0.0, 0.25, 0.5)
        torso.addBox("torso_right", 0.2, 0.0, 1.0, 0.0, -0.25, 0.5)
            
        
        

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.5)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                base.removeCollisionObject("my_front_ground")
                base.removeCollisionObject("my_back_ground")
                base.removeCollisionObject("my_left_ground")
                base.removeCollisionObject("my_right_ground")
                face.removeCollisionObject("face")
                base.removeCollisionObject("base")
                base.removeCollisionObject("base_left")
                base.removeCollisionObject("base_right")
                base.removeCollisionObject("base_front")
                head.removeCollisionObject("head")
                torso.removeCollisionObject("torso_left")
                torso.removeCollisionObject("torso_right")
                                                                
                if move_thread:
                    move_thread.stop()

                # On success quit
                # Stopping the MoveIt thread works, however, the action client
                # does not get shut down, and future tucks will not work.
                # As a work-around, we die and roslaunch will respawn us.
                rospy.signal_shutdown("done")
                sys.exit(0)
                return

    def stop(self):
        if self.client:
            self.client.get_move_action().cancel_goal()
        # Stopping the MoveIt thread works, however, the action client
        # does not get shut down, and future tucks will not work.
        # As a work-around, we die and roslaunch will respawn us.
        rospy.signal_shutdown("failed")
        sys.exit(0)

class TuckArmTeleop:

    def __init__(self):
        self.tuck_button = rospy.get_param("~tuck_button", 6)  # default button is the down button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.tucking = False

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        if self.tucking:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't tuck
                rospy.loginfo("Stopping tuck thread")
                self.tuck_thread.stop()
            return
        try:
            if msg.buttons[self.tuck_button] > 0 and msg.buttons[self.deadman] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    # Tuck the arm
                    self.tucking = True
                    rospy.loginfo("Starting tuck thread")
                    self.tuck_thread = TuckThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("tuck_button is out of range")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tuck the arm, either immediately or as a joystck-controlled server.")
    parser.add_argument("--joystick", action="store_true", help="Run as server that tucks on command from joystick.")
    args, unknown = parser.parse_known_args()

    rospy.init_node("tuck_arm")
    rospy.loginfo("New tuck script running")

    if args.joystick:
        t = TuckArmTeleop()
        rospy.spin()
    else:
        rospy.loginfo("Tucking the arm")
        TuckThread()
