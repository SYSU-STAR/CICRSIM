#! /usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import TwistStamped

cmd_sub = TwistStamped()
def poseFeedback(msg):
    # global cmd_sub
    cmd_sub.twist.linear.x = msg.twist.linear.x
    cmd_sub.twist.linear.y = msg.twist.linear.y
    cmd_sub.twist.linear.z = msg.twist.linear.z
    cmd_sub.twist.angular.z = msg.twist.angular.z
def main():
    rospy.init_node('command_process')
    model_control_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.Subscriber('/position_control',TwistStamped,poseFeedback)
    # global cmd_sub
    pub_pose_msg = ModelState()
    pub_pose_msg.model_name = 'ardrone'
    pub_pose_msg.reference_frame = 'world'
    while not rospy.is_shutdown():
        model_state = get_model_state('ardrone','world')
        pub_pose_msg.pose.position.x = model_state.pose.position.x
        pub_pose_msg.pose.position.y = model_state.pose.position.y
        pub_pose_msg.pose.position.z = model_state.pose.position.z

        pub_pose_msg.pose.orientation.w = model_state.pose.orientation.w
        pub_pose_msg.pose.orientation.x = model_state.pose.orientation.x
        pub_pose_msg.pose.orientation.y = model_state.pose.orientation.y
        pub_pose_msg.pose.orientation.z = model_state.pose.orientation.z

        pub_pose_msg.twist.linear.x = cmd_sub.twist.linear.x
        pub_pose_msg.twist.linear.y = cmd_sub.twist.linear.y
        pub_pose_msg.twist.linear.z = cmd_sub.twist.linear.z


        pub_pose_msg.twist.angular.x = model_state.twist.angular.x
        pub_pose_msg.twist.angular.y = model_state.twist.angular.y
        pub_pose_msg.twist.angular.z = cmd_sub.twist.angular.z
        model_control_pub.publish(pub_pose_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
