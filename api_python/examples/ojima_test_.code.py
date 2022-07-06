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
import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

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
    print(action_list)
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

stop_observe_feedback = False
def observe_feedback(base_cyclic):
    global stop_observe_feedback
    stop_observe_feedfack = False
    while not stop_observe_feedback:
        feedback = base_cyclic.RefreshFeedback()
        print('World Positoin(m) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))
        print('World Eular Rotation(deg) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_theta_x, feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z), end='\n\n')
        #print('Positoin x(m)', feedback.base.tool_pose_x , 'y(m)', feedback.base.tool_pose_y, 'z(m)', feedback.base.tool_pose_z, '\nEular x(deg)', feedback.base.tool_pose_theta_x, 'y(deg)', feedback.base.tool_pose_theta_y, 'z(deg)', feedback.base.tool_pose_theta_z, end='\n\n')
        time.sleep(0.1)

def example_cartesian_action_movement(base, base_cyclic, px, py, pz, eux, euy, euz):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = px        # (meters)
    cartesian_pose.y = py        # (meters)
    cartesian_pose.z = pz        # (meters)
    cartesian_pose.theta_x = eux # (eular degrees)
    cartesian_pose.theta_y = euy # (eular degrees)
    cartesian_pose.theta_z = euz # (eular degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)
    
    global stop_observe_feedback
    stop_observe_feedback = False
    th_observe_feedback = threading.Thread(target=observe_feedback, args=(base_cyclic,))
    th_observe_feedback.start();

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        stop_observe_feedback = True
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
        
    th_observe_feedback.join()
    return finished

def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        feedback = base_cyclic.RefreshFeedback()
        print(feedback.base.tool_pose_x ,feedback.base.tool_pose_y,feedback.base.tool_pose_z,feedback.base.tool_pose_theta_x,feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z)
        
        while 1:
            px, py, pz, eux, euy, euz = map(float, input(">> ").split())
            success &= example_cartesian_action_movement(base, base_cyclic, px, py, pz, eux, euy, euz)
            
        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
