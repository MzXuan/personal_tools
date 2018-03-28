#!/usr/bin/env python
"""
follow trajectory (including velocity, and accerleration) read from 
csv

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


SORT_ORDER = {'shoulder_pan_joint':0, 'shoulder_lift_joint':1, 'elbow_joint':2,
               'wrist_1_joint':3, 'wrist_2_joint':4, 'wrist_3_joint':5}

CSV_name = './data/rrt.csv'
TOPIC_name = '/move_pipeline/calculated_trajectory'


class TrajPublisher():
    def __init__(self):
        global CLIENT
        self.trajectory_list = []
        self.initial_position = []
        self.joint_states = JointState()
        self.traj_pub = rospy.Publisher(TOPIC_name, JointTrajectory, queue_size = 1)


    def callback(self, data):
        self.joint_states.name = data.name
        self.joint_states.position = data.position
        self.joint_states.velocity = data.velocity

    #read trajectory from csv file
    def read_data(self):  
        # open_witdth = RG2()
        # open_witdth.req.target_width = 60
        # self.rg2_client(Float64(60))
        with open(CSV_name, 'rb') as f:
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
                        point.time_from_start = rospy.Duration(next(reader)[0])
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
    
    def pub_traj(self):
        for single_traj in self.trajectory_list:
            self.traj_pub.publish(single_traj)
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('follow_traj', disable_signals=True,anonymous=True)

    try:
        traj_follower = TrajPublisher()
        traj_follower.read_data()
        traj_follower.pub_traj()
        rospy.spin()
        # traj_follower.listener()
    except rospy.ROSInterruptException: pass   
