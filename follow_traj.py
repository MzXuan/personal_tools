#!/usr/bin/env python
"""
follow trajectory (including velocity, and accerleration) read from csv file
"""

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import *
from ur_control.srv import RG2
import csv

import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import sys
import time
import numpy as np

# TRAJ_NAME = '../experiment1/traj_RRT.csv'

SORT_ORDER = {'shoulder_pan_joint':0, 'shoulder_lift_joint':1, 'elbow_joint':2,
               'wrist_1_joint':3, 'wrist_2_joint':4, 'wrist_3_joint':5}
CLIENT = None

def TicTocGenerator():
    # Generator that returns time differences
    ti = 0           # initial time
    tf = time.time() # final time
    while True:
        ti = tf
        tf = time.time()
        yield tf-ti # returns the time difference

TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# This will be the main function through which we define both tic() and toc()
def toc(tempBool=True):
    # Prints the time difference yielded by generator instance TicToc
    tempTimeInterval = next(TicToc)
    if tempBool:
        print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

def tic():
    # Records a time in TicToc, marks the beginning of a time interval
    toc(False)



class TrajFollower():
    def __init__(self):
        global CLIENT
        self.trajectory_list = []
        self.initial_position = []
        self.joint_states = JointState()
        self.subscriber = rospy.Subscriber("joint_states", JointState,self.callback, queue_size = 1)
        rospy.wait_for_service('rg2_gripper/control_width')
        self.rg2_client=rospy.ServiceProxy('rg2_gripper/control_width',RG2)
        # self.release_gripper()
        CLIENT = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        CLIENT.wait_for_server()
        print "Connected to server"
        # self.joint_states_lock = JointState()
        # self.subscriber = rospy.Subscriber("/joint_states", JointState, self.callback, queue_size = 1)
        self.read_data()


    def callback(self, data):
        self.joint_states.name = data.name
        self.joint_states.position = data.position
        self.joint_states.velocity = data.velocity

    #read trajectory from csv file
    def read_data(self):  
        # open_witdth = RG2()
        # open_witdth.req.target_width = 60
        # self.rg2_client(Float64(60))
        with open(TRAJ_NAME, 'rb') as f:
            reader = csv.reader(f,delimiter=',', quotechar='|',quoting=csv.QUOTE_NONNUMERIC)
            while True:
                try:
                    temp = next(reader)
                    #define new traj
                    if "shoulder_pan_joint" in temp:
                        traj=JointTrajectory()
                        print "new traj"                        
                        traj.joint_names = temp
                        t=0
                    elif "end" in temp:
                        self.trajectory_list.append(traj)
                    else:
                        point = JointTrajectoryPoint()
                        point.positions = temp
                        point.velocities = next(reader)
                        point.accelerations = next(reader)
                        point.effort = next(reader)
                        point.time_from_start = rospy.Duration(next(reader)[0])*1.4
                        #for debug use
                        # point.velocities = []
                        # point.accelerations = []
                        # point.time_from_start = rospy.Duration(next(reader)[0]) * 2
                        traj.points.append(point)
                except StopIteration:
                    print "check traj push correct"
                    for traj in self.trajectory_list:
                            print traj.points[0].positions[0]
                    break
    def release_gripper(self):
        try:
            self.rg2_client(Float64(60))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def close_gripper(self):
        try:
            self.rg2_client(Float64(30))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def move_to_initial(self): #sort
        global CLIENT
        js = self.joint_states
        while (not js.name or not js.position):
            rospy.sleep(0.1)
        target_pos = self.trajectory_list[0].points[0].positions
        sort_js = zip(js.name, js.position, js.velocity)
        #print sort_js
        sort_js.sort(key=lambda val: SORT_ORDER[val[0]])
        print "current pos:"
        print sort_js
        print "target pos:"
        print target_pos
        print "move to initial pos"

        current_pos = JointTrajectory() 
        names=[]
        position=[]
        velocity=[]
        for name,pos,vel,in sort_js:
            names.append(name)
            position.append(pos)
            velocity.append(vel)
        temp_grip_pos = [2.1784160137176514, -1.5212090651141565,1.7590484619140625, 4.394067287445068, -1.6253512541400355, -3.9644673506366175]
        traj=JointTrajectory()
        traj2=JointTrajectory()
        traj.joint_names = names
        traj2.joint_names = names
        traj.points.append(JointTrajectoryPoint(positions=position, velocities=[0]*6, time_from_start=rospy.Duration(0)))
        traj.points.append(JointTrajectoryPoint(positions=target_pos, velocities=[0]*6, time_from_start=rospy.Duration(3)))
       # traj.points.append(JointTrajectoryPoint(positions=temp_grip_pos, velocities=[0]*6, time_from_start=rospy.Duration(4)))
        
        traj2.points.append(JointTrajectoryPoint(positions=temp_grip_pos, velocities=[0]*6, time_from_start=rospy.Duration(0)))
        traj2.points.append(JointTrajectoryPoint(positions=target_pos, velocities=[0]*6, time_from_start=rospy.Duration(2)))
        try:            
            self.move(traj)
            rospy.sleep(1)
            #self.release_and_grip()
            # rospy.sleep(1)
            # self.move(traj2)
            # rospy.sleep(1)
            print "Initial DONE"
            return True
        except KeyboardInterrupt:
            return False

            

    #execute trajectory
    def exe_traj(self):
        global CLIENT
        print "I'm about to executing trajectory"
        try:
            inp = raw_input("Continue? y/n: ")[0]
            if (inp == 'y'):
                self.move_to_initial()
                time.sleep(1)
                # self.suspend()
                # time.sleep(5)   
                # for traj in self.trajectory_list:  
                #     self.move(traj)
                #     time.sleep(1)
                try:              
                    while True:                             
                        for traj in self.trajectory_list:  
                            if self.move(traj):
                                time.sleep(1.5)
                            else:                                
                                return
                except KeyboardInterrupt:
                    print('interrupted! and stop all motion') 
                    return
                ##TODO: check keyboard exit###
            else:
                print "Halting program"
        except rospy.ROSInterruptException: pass

    #suspend trajectory
    def suspend(self):
        print ("suspend trajectory")
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            g.trajectory.joint_names = joint_states.name
            joints_pos = joint_states.position
            g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(1)))         
            CLIENT.send_goal(g)
            CLIENT.wait_for_result()
        except KeyboardInterrupt:
            print ('cancel goal by keyboard interrupted')
            CLIENT.cancel_goal()

    def move(self,traj):
        global CLIENT
        print ('move traj')
        g = FollowJointTrajectoryGoal()
        g.trajectory.joint_names = traj.joint_names
        g.trajectory.points = traj.points
        try:
            CLIENT.send_goal(g) 
            tic()
            CLIENT.wait_for_result()
            toc()
            print "done"
            return True
        except KeyboardInterrupt:
            print ('cancel goal by keyboard interrupted')
            CLIENT.cancel_goal()
            return False

    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('follow_traj', disable_signals=True,anonymous=True)

    try:
        traj_follower = TrajFollower()
        traj_follower.exe_traj()
        rospy.spin()
        # traj_follower.listener()
    except rospy.ROSInterruptException: pass   





