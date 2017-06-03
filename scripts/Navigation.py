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

class GPSRNavigate:
    def __init__(self):
        self.result_pub = rospy.Publisher('/gpsr/navigation/result',String)
        self.request_sub = rospy.Subscriber('/gpsr/navigation/input',String,self.NavigateToDestination)
        self.base_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.BaseCB)

        self.location_list = [["bookshelf",0,0,0,0],
                              ["sofa",0,0,0,0],
                              ["tv",0,0,0,0],
                              ["table",0,0,0,0],
                              ["bar table",0,0,0,0],
                              ["table set 1",0,0,0,0],
                              ["table set 2",0,0,0,0]]
        self.robot_pose_x = 999
        self.robot_pose_y = 999
        
    def BaseCB(self,pose):
        self.robot_pose_x = pose.pose.pose.position.x
        self.robot_pose_y = pose.pose.pose.position.y
        
    def NavigateToDestination(self,destination):
        location_num = -1
        for location_num_i in range(len(self.location_list)):
            if self.location_list[location_num_i][0] in destination.data:
                location_num = location_num_i
        if location_num == -1:
            print "not exist such object"
            return

        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'         # 地図座標系
        goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
        goal.target_pose.pose.position.x =  self.location_list[location_num][1]
        goal.target_pose.pose.position.y =  self.location_list[location_num][2]
        q = tf.transformations.quaternion_from_euler(0, 0, self.location_list[location_num][2])
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        ac.send_goal(goal);
        clear_map_cnt = 0
        while(1):
            dist_to_goal = math.hypot(self.robot_pose_x - goal.target_pose.pose.position.x,self.robot_pose_y - goal.target_pose.pose.position.y)
            if dist_to_goal < 0.5:
                break
        result = String()
        result = "finish"
        self.result_pub.publish(result)
        print "finish"
        
        
if __name__ == '__main__':
    rospy.init_node('gpsr_navigate',anonymous=True)
    gpsr_navigate = GPSRNavigate()
    rospy.spin()
    
