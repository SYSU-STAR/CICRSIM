#! /usr/bin/env python
import rospy
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
odom_sub = Odometry()
pub_pose_msg = ModelState()
pub_pose_msg.model_name = 'ardrone'
pub_pose_msg.reference_frame = 'world'
model_control_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

def odomMSGCallBack(msg):
    odom_sub.pose.pose.position.x = msg.pose.pose.position.x
    odom_sub.pose.pose.position.y = msg.pose.pose.position.y
    odom_sub.pose.pose.position.z = msg.pose.pose.position.z

    odom_sub.pose.pose.orientation.w = msg.pose.pose.orientation.w
    odom_sub.pose.pose.orientation.x = msg.pose.pose.orientation.x
    odom_sub.pose.pose.orientation.y = msg.pose.pose.orientation.y
    odom_sub.pose.pose.orientation.z = msg.pose.pose.orientation.z

    odom_sub.twist.twist.linear.x = msg.twist.twist.linear.x
    odom_sub.twist.twist.linear.y = msg.twist.twist.linear.y
    odom_sub.twist.twist.linear.z = msg.twist.twist.linear.z

    odom_sub.twist.twist.angular.x = msg.twist.twist.angular.x
    odom_sub.twist.twist.angular.y = msg.twist.twist.angular.y
    odom_sub.twist.twist.angular.z = msg.twist.twist.angular.z

    pub_pose_msg.pose.position.x = odom_sub.pose.pose.position.x
    pub_pose_msg.pose.position.y = odom_sub.pose.pose.position.y
    pub_pose_msg.pose.position.z = odom_sub.pose.pose.position.z

    pub_pose_msg.pose.orientation.w = odom_sub.pose.pose.orientation.w
    pub_pose_msg.pose.orientation.x = odom_sub.pose.pose.orientation.x
    pub_pose_msg.pose.orientation.y = odom_sub.pose.pose.orientation.y
    pub_pose_msg.pose.orientation.z = odom_sub.pose.pose.orientation.z

    if(odom_sub.twist.twist.linear.x>=3.0):
        pub_pose_msg.twist.linear.x = 3.0
    if(odom_sub.twist.twist.linear.x<3.0):
        pub_pose_msg.twist.linear.x = odom_sub.twist.twist.linear.x
    if(odom_sub.twist.twist.linear.y>=3.0):
        pub_pose_msg.twist.linear.y = 3.0
    if(odom_sub.twist.twist.linear.y<3.0):
        pub_pose_msg.twist.linear.y = odom_sub.twist.twist.linear.y
    if(odom_sub.twist.twist.linear.z>=3.0):
        pub_pose_msg.twist.linear.z = 3.0
    if(odom_sub.twist.twist.linear.z<3.0):
        pub_pose_msg.twist.linear.z = odom_sub.twist.twist.linear.z


    pub_pose_msg.twist.angular.x = odom_sub.twist.twist.angular.x
    pub_pose_msg.twist.angular.y = odom_sub.twist.twist.angular.y
    pub_pose_msg.twist.angular.z = odom_sub.twist.twist.angular.z

    model_control_pub.publish(pub_pose_msg)

def main():
    global take_off_flag
    rospy.init_node('command_process')
    rospy.Subscriber('/position_control',Odometry,odomMSGCallBack)
    rospy.logwarn("command_process Load Sucessfully")
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
