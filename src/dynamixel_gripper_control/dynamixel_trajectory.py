#! /usr/bin/env python
from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

import dynamixel_gripper_control.msg as msg
from pressure_controller_ros.msg import CommandAction, CommandGoal

import time
import sys
import os
import numbers
import numpy as np
import copy
import yaml
import threading


class trajSender:
    def __init__(self, speed_factor = 1.0, name = "robotiq_2f_control"):
        self._name = 'dynamixel'

        action_name = rospy.get_param('~action_name', 'dynamixel')
        self.command_client = actionlib.SimpleActionClient(action_name, CommandAction)

        print('connecting to traj client: %s'%(self._name+'/dynamixel_trajectory'))
        self.traj_client = actionlib.SimpleActionClient(self._name+'/dynamixel_trajectory', msg.TrajectoryAction)
        self.command_client.wait_for_server()
        self.traj_client.wait_for_server()

        if speed_factor <=0.0:
            speed_factor = 1.0

        self.speed_multiplier = 1./speed_factor
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self.send_command("",[],False)
        self.send_command("",[],False)
        self.send_command("on",[],False)

        

    # build the whole trajectory
    def build_traj(self, traj):
        traj_goal = msg.TrajectoryGoal()
        traj_goal.trajectory = msg.Trajectory()

        for entry in traj:
            traj_goal.trajectory.points.append(msg.TrajectoryPoint(
                        pos         = entry[1],
                        speed       = entry[2],
                        force       = entry[3],
                        time_from_start = self.speed_multiplier*rospy.Duration(entry[0])))

        return traj_goal



    def go_to_start(self, traj_goal, reset_time, blocking=True):
        if traj_goal is None:
            return False
        self.start_pos = traj_goal[0][1]

        goal_tmp = msg.TrajectoryGoal()
        goal_tmp.trajectory = msg.Trajectory()

        goal_tmp.trajectory.points.append(msg.TrajectoryPoint(pos=self.start_pos, speed=1023, force=1023, time_from_start=rospy.Duration(traj_goal[0][0])))
        goal_tmp.trajectory.points.append(msg.TrajectoryPoint(pos=self.start_pos, speed=1023, force=1023, time_from_start=rospy.Duration(traj_goal[0][0])))

        self.execute_traj(goal_tmp, blocking)





    def execute_traj(self, traj_goal, blocking=True):
        #try:
        self.traj_client.send_goal(traj_goal)

        print('Goal Sent')

        if blocking:
            self.traj_client.wait_for_result()
        else:
            return self.traj_client

        #except KeyboardInterrupt:
        #    self.traj_client.cancel_goal()
        #except:
        #    raise


            

    def send_command(self, command, args, wait_for_ack=True):
        # Validate inputs
        if not isinstance(command, str):
            raise ValueError('CONFIG: Command must be a string')

        if isinstance(args, list) or isinstance(args, tuple):
            pass
        elif isinstance(args, numbers.Number):
            args=[args]
        else:
            raise ValueError('CONFIG: Args must be a list, tuple, or number')

        if self.DEBUG:
            print(command, args)

        # Send commands to the commader node and wait for things to be taken care of
        goal = CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()

        if not self.command_client.get_result():
            print('Something went wrong and a setting was not validated')
            raise
        else:
            pass



    def shutdown(self, reset=None):
        print('HAND CONTROLLER: Cancel current goals')
        self.traj_client.cancel_all_goals()
           
        if reset is not None:
            print('HAND CONTROLLER: Setting resting position')

            if reset == 'resting':
                out_press= copy.deepcopy(self.start_pos)
                out_press = out_press
                print(out_press)
                self.send_command("set",[0,out_press], False)
                self.send_command("off",[],False)
            else:
                pass
                #self.send_command("set",reset)

        self.command_client.cancel_all_goals()
        




if __name__ == '__main__':
    pass