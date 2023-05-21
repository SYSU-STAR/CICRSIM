#! /usr/bin/env python
import pygame
from pygame.locals import *
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf.transformations as tft
def main():
    rospy.init_node('keyboard_control')
    key_axes = [0, 0, 0, 0, 0, 0, 0, 0] #key value
    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 750, 272)
    screen = pygame.display.set_mode(window_size.size)
    img = pygame.image.load("./files/keyboard_control.png")
    model_odom_pub = rospy.Publisher('/position_control',Odometry,queue_size=10)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    body_pose_msg = ModelState() #uav body pose
    last_body_pose_msg = ModelState() #
    pub_pose_msg = ModelState()
    odom = Odometry()
    rospy.logwarn("keyboard_control Load Sucessfully")
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        screen.blit(img, (1,1))
        pygame.display.flip()
        drone_state = get_model_state('ardrone','world')
        for event in pygame.event.get():
            if event.type == KEYDOWN: 
                if event.key == pygame.K_UP:
                    key_axes[0] = 1
                if event.key == pygame.K_DOWN:
                    key_axes[1] = 1
                if event.key == pygame.K_LEFT:
                    key_axes[2] = 1
                if event.key == pygame.K_RIGHT:
                    key_axes[3] = 1
                if event.key == pygame.K_w:
                    key_axes[4] = 1
                if event.key == pygame.K_s:
                    key_axes[5] = 1
                if event.key == pygame.K_a:
                    key_axes[6] = 1
                if event.key == pygame.K_d:
                    key_axes[7] = 1

            # when keyup, reset velcity
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_UP:
                    key_axes[0] = 0
                if event.key == pygame.K_DOWN:
                    key_axes[1] = 0
                if event.key == pygame.K_LEFT:
                    key_axes[2] = 0
                if event.key == pygame.K_RIGHT:
                    key_axes[3] = 0
                if event.key == pygame.K_w:
                    key_axes[4] = 0
                if event.key == pygame.K_s:
                    key_axes[5] = 0
                if event.key == pygame.K_a:
                    key_axes[6] = 0
                if event.key == pygame.K_d:
                    key_axes[7] = 0

        ######
        if(key_axes[0]==1 and key_axes[1]==0):
            body_pose_msg.twist.linear.x = 1
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x
        if(key_axes[0]==0 and key_axes[1]==0):
            body_pose_msg.twist.linear.x = 0 
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x
        if(key_axes[1]==1 and key_axes[0]==0):
            body_pose_msg.twist.linear.x = -1
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x
        if(key_axes[1]==0 and key_axes[0]!=1):
            body_pose_msg.twist.linear.x = 0 
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x

        if(key_axes[2]==1 and key_axes[3]==0):
            body_pose_msg.twist.linear.y = 1
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y
        if(key_axes[2]==0 and key_axes[3]==0):
            body_pose_msg.twist.linear.y = 0
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y    
        if(key_axes[3]==1 and key_axes[2]==0):
            body_pose_msg.twist.linear.y = -1
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y
        if(key_axes[3]==0 and key_axes[2]!=1):
            body_pose_msg.twist.linear.y = 0
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y    

        if(key_axes[4]==1 and key_axes[5]==0):
            body_pose_msg.twist.linear.z = 30
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z
        if(key_axes[4]==0 and key_axes[5]==0):
            body_pose_msg.twist.linear.z = 0
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z  
        if(key_axes[5]==1 and key_axes[4]==0):
            body_pose_msg.twist.linear.z = -1
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z
        if(key_axes[5]==0 and key_axes[4]!=1):
            body_pose_msg.twist.linear.z = 0
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z  

        if(key_axes[6]==1 and key_axes[7]==0):
            body_pose_msg.twist.angular.z = 1
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z
        if(key_axes[6]==0 and key_axes[7]==0):
            body_pose_msg.twist.angular.z = 0
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z  
        if(key_axes[7]==1 and key_axes[6]==0):
            body_pose_msg.twist.angular.z = -1
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z
        if(key_axes[7]==0 and key_axes[6]!=1):
            body_pose_msg.twist.angular.z = 0
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z  
        
        # orientation ---> euler
        quaternion_uav = (drone_state.pose.orientation.w, drone_state.pose.orientation.x, drone_state.pose.orientation.y, drone_state.pose.orientation.z)
        euler_angles = tft.euler_from_quaternion(quaternion_uav, 'sxyz') # yaw
        if(euler_angles[0]>=0):
            uav_yaw = math.pi - euler_angles[0]
        if(euler_angles[0]<0):
            uav_yaw = -math.pi - euler_angles[0]

        # body ---> world
        rotation_matrix_z = np.matrix([[math.cos(uav_yaw),-math.sin(uav_yaw),0],[math.sin(uav_yaw),math.cos(uav_yaw),0],[0,0,1]])
        body_vel_matrix = np.matrix([last_body_pose_msg.twist.linear.x, last_body_pose_msg.twist.linear.y, last_body_pose_msg.twist.linear.z])
        body_vel_matrix_trans = body_vel_matrix.T
        world_vel_matrix = rotation_matrix_z.dot(body_vel_matrix_trans)

        #######################################################
        odom.pose.pose.position.x = drone_state.pose.position.x
        odom.pose.pose.position.y = drone_state.pose.position.y
        odom.pose.pose.position.z = drone_state.pose.position.z

        odom.pose.pose.orientation.w = drone_state.pose.orientation.w
        odom.pose.pose.orientation.x = drone_state.pose.orientation.x
        odom.pose.pose.orientation.y = drone_state.pose.orientation.y
        odom.pose.pose.orientation.z = drone_state.pose.orientation.z

        odom.twist.twist.linear.x = world_vel_matrix[0]
        odom.twist.twist.linear.y = world_vel_matrix[1]
        odom.twist.twist.linear.z = world_vel_matrix[2]

        odom.twist.twist.angular.x = drone_state.twist.angular.x
        odom.twist.twist.angular.y = drone_state.twist.angular.y
        odom.twist.twist.angular.z = last_body_pose_msg.twist.angular.z

        model_odom_pub.publish(odom)

    





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


