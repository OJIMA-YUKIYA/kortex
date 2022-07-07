#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
#すべて差分
import sys
import os
import time
import threading
import serial
import numpy  as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

buff = [0, 0, 0, 0, 0, 0, 0]

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Retract":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_action_movement(base, base_cyclic, dx, dy, dz,theta_x,theta_y,theta_z):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x + dx         # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y + dy    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z + dz    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + theta_x # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + theta_y# (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + theta_z#(degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

#from 106 01-gripper_command.py
class GripperCommandExample:
    def __init__(self, router, base,proportional_gain = 2.0):


        self.proportional_gain = proportional_gain
        self.router = router
        self.base = base

    def Goto(self, target_position):

        if target_position > 1.0:
            target_position = 1.0
        if target_position < 0.0:
            target_position = 0.0

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = target_position
        print("Going to position {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)
    

def normalize(recive):
    for i in range(3):
        recive[i] = float(recive[i])/127
    for i in range(3,6):
        recive[i] = float(recive[i])/127*180
    recive[6] = float(recive[6])+127
    recive[6] /= 255

def main():
    global buff
    recive = np.array([0,0,0,0,0,0,0])
    
    readSerial = serial.Serial("../serial_out",9600)
    
    # Import the utilities helper module
    #
    import argparse

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    #
    parser = argparse.ArgumentParser()

    args = utilities.parseConnectionArguments(parser)#
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        #
        example = GripperCommandExample(router, base)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        feedback = base_cyclic.RefreshFeedback()
        print(feedback.base.tool_pose_x ,feedback.base.tool_pose_y,feedback.base.tool_pose_z,feedback.base.tool_pose_theta_x,feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z)
        while 1:

            buff = readSerial.read(7)

            for i in range(7):
                recive[i] = np.array(buff[i],dtype ='int8')
            
            recive = recive.astype(np.float16)
            
            normalize(recive)

            dx, dy, dz, theta_x, theta_y, theta_z, target_position = recive[0], recive[1], recive[2], recive[3], recive[4], recive[5], recive[6]
            
            print(dx, dy, dz, theta_x, theta_y, theta_z, target_position, "-----------------")
            
            success &= example_cartesian_action_movement(base, base_cyclic,dx, dy, dz, theta_x, theta_y, theta_z)
            feedback = base_cyclic.RefreshFeedback()

            example.Goto(target_position)

            print(feedback.base.tool_pose_x ,feedback.base.tool_pose_y,feedback.base.tool_pose_z,feedback.base.tool_pose_theta_x,feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z)

        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1





if __name__ == "__main__":
    exit(main())
