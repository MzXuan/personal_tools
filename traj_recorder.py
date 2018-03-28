#!/usr/bin/env python
"""
save Trajectory from ros topic

"""

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
#from keyboard.msg import Key
#from control_msgs.msg import *


import csv
import copy
import sys
import time
import numpy as np
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import random

CSV_name = './data/test.csv'
TOPIC_name = '/move_pipeline/calculated_trajectory'


class TrajRecorder():
    def __init__(self):
    	global TOPIC_name
        self.key_press = False
        self.trajectory_list=[]
        self.joint_names=[]
        self.lock = False
        self.traj_sub = rospy.Subscriber(TOPIC_name, JointTrajectory, self.callback, queue_size = 1)
        #self.key_sub = rospy.Subscriber("/keyboard/keydown", Key, self.callback_key, queue_size = 1)

    def callback(self,data):
        ###NOTE: do not write joint trajectory directly, or there might be wired problems###
        
        print "received msg, saving"
        if self.lock == False:
	        self.joint_names = data.joint_names
	        self.trajectory_list.append(data.points)
	        self.lock = True
	        print 'name: ', self.joint_names
	        print 'pos:', data.points[0].positions
	        print "callback"
	        self.write_csv()
	        self.lock= False

        # # when new trajectory received save it to file
        # print "received msg, saving"
        # if not self.trajectory_lock:
        #     self.trajectory_lock.joint_names = copy.copy(data.joint_names)
        #     print 'name: ', self.trajectory_lock.joint_names
        #     #self.trajectory_lock.points=[]
        #     self.trajectory_lock.points = copy.copy(data.points)
        #     print 'pos:', self.trajectory_lock.points[0].positions
        #     self.trajectory_list.append(self.trajectory_lock)
        #     print "callback1"
        # else:
        #     self.trajectory_lock2.joint_names = copy.copy(data.joint_names)
        #     print 'name: ', self.trajectory_lock2.joint_names
        #     #self.trajectory_lock.points=[]
        #     self.trajectory_lock2.points = copy.copy(data.points)
        #     print 'pos:', self.trajectory_lock2.points[0].positions
        #     self.trajectory_list.append(self.trajectory_lock2)
        #     print "callback2"
        # print ' ----------------------- '
        # print self.trajectory_list
        # print ' ----------------------- '

    def write_csv(self):
        #press a key to save
        global CSV_name
        with open(CSV_name, 'w') as csvfile: pass
        # print self.trajectory_list
        for points in self.trajectory_list:
            with open(CSV_name, 'a') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_NONNUMERIC)
                joint_name = self.joint_names
                spamwriter.writerow(joint_name)
                for point in points:                        
                    spamwriter.writerow(point.positions)
                    spamwriter.writerow(point.velocities)
                    spamwriter.writerow(point.accelerations)
                    spamwriter.writerow(point.effort)
                    spamwriter.writerow([point.time_from_start.to_sec()])
                spamwriter.writerow(["end"])
        print "save successful"

    def listener(self):
        rospy.spin()      

if __name__ == '__main__':
    rospy.init_node('Traj_recorder',anonymous=True)
    print "Start recording trajectory, waiting for topics..."
    try:
        traj_recorder = TrajRecorder()
        traj_recorder.listener()
        
    except rospy.ROSInterruptException: pass 
