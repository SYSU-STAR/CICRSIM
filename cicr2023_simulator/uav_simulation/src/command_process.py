#! /usr/bin/env python
import rospy
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import datetime

odom_sub = Odometry()
pub_pose_msg = ModelState()
pub_pose_msg.model_name = 'ardrone'
pub_pose_msg.reference_frame = 'world'
model_control_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

v_last_x = 0.0
v_last_y = 0.0
v_last_z = 0.0
w_last_x = 0.0
w_last_y = 0.0
w_last_z = 0.0
acc_x = 0.0
acc_y = 0.0
acc_z = 0.0
acc_w_x=0.0
acc_w_y=0.0
acc_w_z=0.0

def odomMSGCallBack(msg):
    global t_last
    global v_last_x
    global v_last_y
    global v_last_z
    global w_last_x
    global w_last_y
    global w_last_z
    global acc_x
    global acc_y
    global acc_z
    global acc_w_x
    global acc_w_y
    global acc_w_z

    odom_sub.pose.pose.position.x = msg.pose.pose.position.x
    odom_sub.pose.pose.position.y = msg.pose.pose.position.y
    odom_sub.pose.pose.position.z = msg.pose.pose.position.z

    odom_sub.pose.pose.orientation.w = msg.pose.pose.orientation.w
    odom_sub.pose.pose.orientation.x = msg.pose.pose.orientation.x
    odom_sub.pose.pose.orientation.y = msg.pose.pose.orientation.y
    odom_sub.pose.pose.orientation.z = msg.pose.pose.orientation.z

    # linear vel
    odom_sub.twist.twist.linear.x = msg.twist.twist.linear.x
    odom_sub.twist.twist.linear.y = msg.twist.twist.linear.y
    odom_sub.twist.twist.linear.z = msg.twist.twist.linear.z

    # angular vel
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

    t_now = rospy.Time.now()
    dt = (t_now - t_last).to_sec()
    v_now_x = odom_sub.twist.twist.linear.x
    v_now_y = odom_sub.twist.twist.linear.y
    v_now_z = odom_sub.twist.twist.linear.z
    w_now_x = odom_sub.twist.twist.angular.x
    w_now_y = odom_sub.twist.twist.angular.y
    w_now_z = odom_sub.twist.twist.angular.z

    if dt > 0:
        acc_x = (v_now_x - v_last_x) / dt
        acc_y = (v_now_y - v_last_y) / dt
        acc_z = (v_now_z - v_last_z) / dt
        acc_w_x = (w_now_x - w_last_x) / dt
        acc_w_y = (w_now_y - w_last_y) / dt
        acc_w_z = (w_now_z - w_last_z) / dt


    total_acc = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    if total_acc > 2.0:
        acc_x *= 2.0 / total_acc
        acc_y *= 2.0 / total_acc
        acc_z *= 2.0 / total_acc
    
    v_now_x = v_last_x + acc_x * dt
    v_now_y = v_last_y + acc_y * dt
    v_now_z = v_last_z + acc_z * dt

    total_vel = math.sqrt(v_now_x**2 + v_now_y**2 + v_now_z**2)
    if total_vel > 3.0:
        v_now_x *= 3.0 / total_vel
        v_now_y *= 3.0 / total_vel
        v_now_z *= 3.0 / total_vel

    total_w_acc = math.sqrt(acc_w_x**2 + acc_w_y**2 + acc_w_z**2)
    if total_w_acc > 2.0:
        acc_w_x *= 2.0 / total_w_acc
        acc_w_y *= 2.0 / total_w_acc
        acc_w_z *= 2.0 / total_w_acc

    w_now_x = w_last_x + acc_w_x * dt
    w_now_y = w_last_y + acc_w_y * dt
    w_now_z = w_last_z + acc_w_z * dt

    total_w = math.sqrt(w_now_x**2 + w_now_y**2 + w_now_z**2)
    if total_w > 1.0:
        w_now_x *= 1.0 / total_w
        w_now_y *= 1.0 / total_w
        w_now_z *= 1.0 / total_w
    
    t_last = t_now
    v_last_x = v_now_x
    v_last_y = v_now_y
    v_last_z = v_now_z
    w_last_x = w_now_x
    w_last_y = w_now_y
    w_last_z = w_now_z


    pub_pose_msg.twist.linear.x = v_now_x
    pub_pose_msg.twist.linear.y = v_now_y
    pub_pose_msg.twist.linear.z = v_now_z



    pub_pose_msg.twist.angular.x = w_now_x
    pub_pose_msg.twist.angular.y = w_now_y
    pub_pose_msg.twist.angular.z = w_now_z

    model_control_pub.publish(pub_pose_msg)

def main():
    rospy.init_node('command_process')
    rospy.Subscriber('/position_control',Odometry,odomMSGCallBack)
    global t_last
    t_last = rospy.Time.now()
    rospy.logwarn("command_process Load Sucessfully")
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
