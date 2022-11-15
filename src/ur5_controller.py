#!/usr/bin/env python3
import rospy
import time
import roslaunch
from universal_robots.msg import position
from universal_robots.msg import joint
from universal_robots.msg import position_command
from universal_robots.msg import joint_command
import numpy
from numpy import pi
import math

class ur5_command_publisher:
    def __init__(self,timeout=60):
    # rospy.init_node('ur5_node')
        self.pose_publisher = rospy.Publisher('ur5_control_pose_level', position_command, queue_size=1,latch=True)
        self.joint_publisher = rospy.Publisher('ur5_control_joint_level', joint_command, queue_size=1,latch=True)
        
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            ur5_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/dongting/Documents/catkin_ws/src/code_idealab_ros/src/universal_robots/launch/ur5_control.launch"])
            ur5_launch.start()
            time.sleep(5)
        except:
            rospy.loginfo("cant not start ur5")
        time_a = time.time()
        while (self.pose_publisher.get_num_connections()==0) and (self.joint_publisher.get_num_connections()==0):
            rospy.loginfo("waiting for sub connection")
            time.sleep(1)
            if time.time()-time_a >timeout:
                rospy.loginfo("Connection Timeout")
                break
        rospy.loginfo("connected to subscriber")
        # self.pose_subscriber = rospy.Subscriber('ur5_pose_publisher',position,self.pose_callback)
    
    # def pose_callback(self,data):
    #     is_robot_moving = data.is_moving
    #     if is_robot_moving = 
              
    def rotate_ur5_ros(self,RPY,vel,acc=1,wait=True):
        pos_to_pub = position_command()
        pos_to_pub.pos = [0,0,0]
        pos_to_pub.RPY = RPY
        pos_to_pub.vel = vel
        pos_to_pub.acc = acc
        pos_to_pub.wait_flag = wait
        self.pose_publisher.publish(pos_to_pub)
        time.sleep(5)
        
    def movel_ur5_ros(self,pos,vel,acc=1,wait=True):
        pos_to_pub = position_command()
        pos_to_pub.pos = pos
        pos_to_pub.RPY = [0,0,0]
        pos_to_pub.vel = vel
        pos_to_pub.acc = acc
        pos_to_pub.wait_flag = wait
        self.pose_publisher.publish(pos_to_pub)
        distance = numpy.sqrt(numpy.sum(numpy.square(pos)))
        wait_time = distance/vel
        time.sleep(wait_time*1.5)


    def movej_ur5_ros(self,joint_angles,vel,acc=1,wait=True):
        joint_to_pub = joint_command()
        joint_to_pub.joint_angles = joint_angles
        joint_to_pub.vel = vel
        joint_to_pub.acc = acc
        joint_to_pub.wait_flag = wait
        self.joint_publisher.publish(joint_to_pub)
        time.sleep(10)

    def shake_a_clean(self):
        import time
        shake_vel = 0.1
        for item in range(2):
            clean_angle  = numpy.array([pi/4,0,0])
            self.rotate_ur5_ros(clean_angle,shake_vel,1,True)
            time.sleep(2)
            self.rotate_ur5_ros(-2*clean_angle,shake_vel,1,True)
            time.sleep(2)
            self.rotate_ur5_ros(clean_angle,shake_vel,1,True)
            time.sleep(2)
            clean_angle  = numpy.array([0,pi/4,0])
            self.rotate_ur5_ros(clean_angle,shake_vel,1,True)
            time.sleep(2)
            self.rotate_ur5_ros(-2*clean_angle,shake_vel,1,True)
            time.sleep(2)
            self.rotate_ur5_ros(clean_angle,shake_vel,1,True)
            time.sleep(2)


if __name__ == "__main__":
    rospy.init_node('ur5_control_node', anonymous=False)
    ur5 = ur5_command_publisher()

    x_deg = 10
    y_deg = 10
    z_deg = 10

    moving_vector_left = numpy.array((1,1,0))*math.sqrt(2)/2
    moving_vector_right = -numpy.array((1,1,0))*math.sqrt(2)/2
    moving_vector_forward = numpy.array((1,-1,0))*math.sqrt(2)/2
    moving_vector_backward = numpy.array((-1,1,0))*math.sqrt(2)/2
    moving_vector_up = numpy.array((0,0,1))
    moving_vector_down = numpy.array((0,0,-1))

    exp_x_angle  = numpy.array([pi/180*x_deg,0,0])
    exp_z_angle  = numpy.array([0,pi/180*y_deg,0])
    exp_y_angle  = numpy.array([0,0,pi/180*z_deg])

#    prepare_pos_0 = [-1.4427674452411097,-1.3111127058612269,1.8012299537658691,-2.058236900960104,-1.5705226103412073,5.7142510414123535]
#    ur5.movej_ur5_ros(prepare_pos_0,0.05,1,True)

    ur5.rotate_ur5_ros(exp_x_angle,0.03,acc=1,wait=True)
    ur5.rotate_ur5_ros(exp_y_angle,0.03,acc=1,wait=True)
    ur5.rotate_ur5_ros(exp_z_angle,0.03,acc=1,wait=True)

    ur5.movel_ur5_ros(moving_vector_backward*0.05,0.01,1,True)

    time.sleep(5)

    ur5.movel_ur5_ros(moving_vector_backward*-0.05,0.01,1,True)  
    ur5.rotate_ur5_ros(exp_z_angle*-1,0.03,acc=1,wait=True)
    ur5.rotate_ur5_ros(exp_y_angle*-1,0.03,acc=1,wait=True)
    ur5.rotate_ur5_ros(exp_x_angle*-1,0.03,acc=1,wait=True)

    
    time.sleep(5)
    ur5.shake_a_clean()

    rospy.spin()
    # ur5.movel_ur5_ros([0,0,0.05],0.01,1,True)  
    # ur5.movel_ur5_ros([0,0,0.-0.05],0.01,1,True)  
    # prepare_pos_0 = [-1.4427674452411097,-1.3111127058612269,1.8012299537658691,-2.058236900960104,-1.5705226103412073,5.7142510414123535]
    # ur5.movej_ur5_ros(prepare_pos_0,0.05,1,True)
    # ur5.shake_a_clean()    
#    ur5.movel_ur5_ros([0,0,.05],0.01,1,True)  
#    
#    
#    # Test Rotate
#    ur5.rotate_ur5_ros([0,0,-pi/6],0.03,1,True)  
#    ur5.rotate_ur5_ros([0,0,pi/6],0.03,1,True)  
#    # Test cleaning method