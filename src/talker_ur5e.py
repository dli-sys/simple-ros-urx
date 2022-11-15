#!/usr/bin/env python3
import rospy
import sys
from universal_robots.msg import position
from universal_robots.msg import joint
from universal_robots.msg import position_command
from universal_robots.msg import joint_command
import math3d as m3d
import urx
import time
import numpy

"""
A lightweight urx-based ur5 ros publisher and controller
Author: dongting@asu.edu
"""

"""
joint.msg
float64[6] joint_angles
"""

class ur5_pub_sub:
    def __init__(self, ur5_port,tcp=((0,0,0.1476,0,0,0)),payload_m=0.61,payload_location=(0,0,0)):
#        tcp = ((0,0,0,0,0,0))   
#        payload_m = 1   
#        payload_location = (0,0,0.5)   
        try: 
            print("connecting to ur5")
            self.ur5 = urx.Robot(ur5_port)
            print(self.ur5)
            time.sleep(5)
        except: 
            print("Can not connect, check connection and try again")
            pass  
        if self.ur5.host == ur5_port:
            self.ur5.set_tcp(tcp)
            self.ur5.set_payload(payload_m, payload_location)   
        rospy.init_node('ur5_pub_node', anonymous=False)
        self.pose_subscriber = rospy.Subscriber('ur5_control_pose_level', position_command, self.callback)
        self.joint_subscriber = rospy.Subscriber('ur5_control_joint_level', joint_command, self.callback_joint_level)

    # def pose_publisher(self):
    #     self.publisher = rospy.Publisher('ur5_pose_publisher', position, queue_size=10)
    #     rate = rospy.Rate(1000) 
    #     while not rospy.is_shutdown():  
    #         pose= self.ur5.get_pose()   
    #         orient = pose.get_orient()  
    #         q = orient.unit_quaternion  
    #         data = position()
    #         data.q[0] = q[0]
    #         data.q[1] = q[1]
    #         data.q[2] = q[2]
    #         data.q[3] = q[3]
    #         data.p[0] = pose.pos[0]
    #         data.p[1] = pose.pos[1]
    #         data.p[2] = pose.pos[2]
    #         data.is_moving = self.ur5.is_program_running()
    #         self.publisher.publish(data)
    #         rate.sleep()
    
    # def joint_publisher(self):
    #     self.joint_publisher = rospy.Publisher('ur5_joint_publisher', joint, queue_size=10)
    #     rate = rospy.Rate(1000) 
    #     while not rospy.is_shutdown():  
    #         joint_angles= self.ur5.getj()   
    #         data = joint()
    #         data.joint_angles[:] = joint_angles[:]
    #         data.is_moving = self.ur5.is_program_running()
    #         self.joint_publisher.publish(data)
    #         rate.sleep()
    def pose_publisher(self):
        self.pose_publisher = rospy.Publisher('ur5_pose_publisher', position, queue_size=10)
        self.joint_publisher = rospy.Publisher('ur5_joint_publisher', joint, queue_size=10)
        rate = rospy.Rate(1000) 
        while not rospy.is_shutdown():  
            pose= self.ur5.get_pose()   
            orient = pose.get_orient()  
            q = orient.unit_quaternion  
            data = position()
            data.q[0] = q[0]
            data.q[1] = q[1]
            data.q[2] = q[2]
            data.q[3] = q[3]
            data.p[0] = pose.pos[0]
            data.p[1] = pose.pos[1]
            data.p[2] = pose.pos[2]
            data.is_moving = self.ur5.is_program_running()
            self.pose_publisher.publish(data)

            joint_angles= self.ur5.getj()   
            data = joint()
            data.joint_angles[:] = joint_angles[:]
            data.is_moving = self.ur5.is_program_running()
            self.joint_publisher.publish(data)

            rate.sleep()
    
    # def joint_publisher(self):
    #     self.joint_publisher = rospy.Publisher('ur5_joint_publisher', joint, queue_size=10)
    #     rate = rospy.Rate(1000) 
    #     while not rospy.is_shutdown():  
    #         joint_angles= self.ur5.getj()   
    #         data = joint()
    #         data.joint_angles[:] = joint_angles[:]
    #         data.is_moving = self.ur5.is_program_running()
    #         self.joint_publisher.publish(data)
    #         rate.sleep()


    ## position-level control
    def movel_ros(self,vector,v,a,w,m_type):
        # print(m_type)
        if m_type == 'tran':
            current_pose = self.ur5.get_pose()  
            current_pose.pos[:] += vector
            # self.ur5.movel(current_pose,vel=v,acc=a,wait=w,threshold=None, timeout=5)
            self.ur5.movel(current_pose,vel=v,acc=a,wait=w)

        else:    
            current_pose = self.ur5.get_pose()
            Tct = m3d.Transform()
            Tct.pos = m3d.Vector(0,0,0)
            Tct.orient = m3d.Orientation.new_euler(vector, encoding='XYZ')
            new_pos = current_pose*Tct
            # self.ur5.movel(new_pos,vel=v,acc=a,wait=w,threshold=None, timeout=5)
            # distance = numpy.sqrt(numpy.sum(numpy.square(vector)))
            self.ur5.movel(new_pos,vel=v,acc=a,wait=w)

    def callback(self,command):
        # print("in callback")
        pos_vector = numpy.array(command.pos)
        ori_vector = numpy.array(command.RPY)
        vel = command.vel
        acc = command.acc
        wait_flag = command.wait_flag
        if all(v == 0 for v in ori_vector):
            m_type = 'tran'
            vector = pos_vector
        else:
            m_type = 'rotate'
            vector = ori_vector
        self.movel_ros(vector,vel,acc,wait_flag,m_type)
        pass
    ## end position level control

    ## joint level control
    def movej_ros(self,joints,v,a,w):
        self.ur5.movej(joints,vel=v,acc=a,wait=w)

    def callback_joint_level(self,command):
        # print(command)
        joints = numpy.array(command.joint_angles)
        vel = command.vel
        acc = command.acc
        wait_flag = command.wait_flag
        self.movej_ros(joints,vel,acc,wait_flag)
        pass
        

    ## end joint level control

if __name__ == '__main__':  
    try:
        rospy.get_published_topics()
        ros_running = True
    except ConnectionRefusedError as e:
        ros_running = False
if ros_running:
    try:
        ur5_port = "192.168.1.104"
        ur5 = ur5_pub_sub(ur5_port)
        ur5.pose_publisher()
        # ur5.joint_publisher()
    except rospy.ROSInterruptException:
        pass