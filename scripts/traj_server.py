#!/usr/bin/env python

import rospy
import time
import actionlib
import numbers
from scipy.interpolate import interp1d


#Import the specific messages that we created in our tutorials folder.
import dynamixel_gripper_control.msg as msg

from pressure_controller_ros.msg import PressureTrajectoryAction, CommandAction, CommandGoal, PressureTrajectoryGoal, PressureTrajectory, PressureTrajectoryPoint



class TrajAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.TrajectoryFeedback()
    _result = msg.TrajectoryResult()

    def __init__(self, name):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = rospy.get_param(rospy.get_name()+'/action_name','command_dynamixel_action')
        self.controller_rate=rospy.get_param(rospy.get_name()+'/traj_server_rate',50)

        #action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.command_client = actionlib.SimpleActionClient(self._action_name, CommandAction)
        #self.traj_client = actionlib.SimpleActionClient(self._action_name+'/pressure_trajectory', PressureTrajectoryAction)

        self.command_client.wait_for_server()
        #self.traj_client.wait_for_server()

        # Start an actionlib server
        self._as = actionlib.SimpleActionServer(self._action_name+'/dynamixel_trajectory', msg.TrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        #time.sleep(2.0)

        #self.send_command('on',[], True)

    def send_point(self, point):
        if self.DEBUG:
            print("POINT:",point)
        self.send_command('cont',1, True)
        self.send_command('speed',point.speed,True)
        self.send_command('torque',point.force,True)
        self.send_command('set',[0,point.pos], True)


    def execute_cb(self, goal):
        traj= goal.trajectory.points       

        # Put all of the setpoints and durations into numpy arrays for later use
        self.traj_points=[]
        self.traj_times=[]
        for point in traj:
            self.traj_points.append(point.pos)
            self.traj_times.append(point.time_from_start.to_sec())

        if self.DEBUG:
            print(self.traj_points)
            print(self.traj_times)

        #traj_interp = interp1d(self.traj_times,self.traj_points, axis=0)

        # helper variables
        r = rospy.Rate(self.controller_rate)
        success = False
        
        # Initiatilize the feedback
        self._feedback.current_time = 0
        self._feedback.success = True

        # Send the pressure trajectory in real time
        start_time=rospy.Time.now()
        curr_time = rospy.Duration(0.0)

        idx=0

        cur_pt_idx = 0
        print('Starting Trajectory')

        #self.send_command('_flush',[], False)
        self.send_point(traj[0])
        execute_action = [0]*len(traj)

        while curr_time.to_sec() < self.traj_times[-1] and not rospy.is_shutdown():
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self._feedback.success = False
                break
            else:
                # Interpolate point to send from the trajectory given
                curr_time = rospy.Time.now()-start_time

                if curr_time.to_sec() >= self.traj_times[-1] : 
                    break

                curr_time_sec = curr_time.to_sec()
                curr_time_pt = traj[cur_pt_idx].time_from_start.to_sec()
                

                if curr_time_sec>=curr_time_pt:
                    # Set gripper settings if they haven't been sent yet
                    if execute_action[cur_pt_idx]==0:
                        self.send_point(traj[cur_pt_idx])
                        execute_action[cur_pt_idx]=1

                        cur_pt_idx+=1 # Update the index to move on to the next point



                # Update the server
                self._feedback.current_time  = curr_time
                self._as.publish_feedback(self._feedback)

                r.sleep()
                idx += 1

        if self.DEBUG:
            print('Total Iterations: %d'%(idx))

        # Send the ending point to make sure.
        self.send_point(traj[-1])


        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            #rospy.loginfo("End: %s"%(goal.command))




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

    def shutdown(self):
        pass
        #self.comms.shutdown()


        
if __name__ == '__main__':
    try:
        rospy.init_node('pressure_control', disable_signals=True)
        print("TRAJECTORY SERVER: Node Initiatilized (%s)"%(rospy.get_name()))
        server = TrajAction(rospy.get_name())
        print("TRAJECTORY SERVER: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("TRAJECTORY SERVER: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("TRAJECTORY SERVER: Shutting Down")
        server.shutdown()

    