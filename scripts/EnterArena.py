#!/usr/bin/env python
# -*- coding: utf-8 -*
import sys
import copy
import rospy
import math
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Quaternion,PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

class DoorOpenDetecter:
    def __init__(self):
        self.result_pub = rospy.Publisher('/speech_control',String)
        self.base_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.BaseCB)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.LaserCB)

        self.robot_pose_x = 999
        self.robot_pose_y = 999
        self.laser_dist = 0
        
    def BaseCB(self,pose):
        self.robot_pose_x = pose.pose.pose.position.x
        self.robot_pose_y = pose.pose.pose.position.y

    def LaserCB(self,laser_input):
        self.laser_dist = laser_input.ranges[341]
        
    def DoorOpenDetect(self):
        while not rospy.is_shutdown():
            print self.laser_dist
            if self.laser_dist > 0.5:
                break;
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'         # 地図座標系
        goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
        goal.target_pose.pose.position.x =  0
        goal.target_pose.pose.position.y =  0
        q = tf.transformations.quaternion_from_euler(0, 0, 999)
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        ac.send_goal(goal);
        clear_map_cnt = 0
        while not rospy.is_shutdown():
            dist_to_goal = math.hypot(self.robot_pose_x - goal.target_pose.pose.position.x,self.robot_pose_y - goal.target_pose.pose.position.y)
            if dist_to_goal < 0.3:
                break
        result = String()
        result = "speak"
        self.result_pub.publish(result)
        print "finish"
        
        
if __name__ == '__main__':
    rospy.init_node('gpsr_navigate',anonymous=True)
    door_open_detector = DoorOpenDetecter()
    door_open_detector.DoorOpenDetect()
    rospy.spin()
    
