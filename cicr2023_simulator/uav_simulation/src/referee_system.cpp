#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <nav_msgs/Odometry.h>
#include "referee_msgs/Apriltag_info.h"
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
using namespace std;
ros::Subscriber rcv_start_flag, rcv_tag_info;
ros::ServiceClient client;
int tags_num = 8;
int minutes = 3;
int seconds = 0; 
int total_score;
bool start_flag = false;
double detect_error = 0.10;
bool finish_detection = false;
gazebo_msgs::ModelState model_pose, last_model_pose;
vector<geometry_msgs::PoseStamped>real_apriltag_pose(tags_num);
vector<geometry_msgs::PoseStamped>player_apriltag_pose(tags_num);
vector<bool>tag_flag = {false,false,false,false,false,false,false,false};
vector<int>tag_score = {0,0,0,0,0,0,0,0};
ros::Time start_time, end_time;
void ApriltagInfo()
{
    vector<gazebo_msgs::GetModelState> apriltag(tags_num);
    vector<string> apriltag_names = {"static_apriltag_0","static_apriltag_1","static_apriltag_2","static_apriltag_3","static_apriltag_4","static_apriltag_5","static_apriltag_6","static_apriltag_7"};
    for(int i=0;i<tags_num;i++)
    {
        apriltag[i].request.model_name= apriltag_names[i];
        apriltag[i].request.relative_entity_name = "world";
        if(client.call(apriltag[i]))
        {
            real_apriltag_pose[i].pose.position.x = apriltag[i].response.pose.position.x;
            real_apriltag_pose[i].pose.position.y = apriltag[i].response.pose.position.y;
            real_apriltag_pose[i].pose.position.z = apriltag[i].response.pose.position.z;
        }
    }
}
void Time_Count(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        start_flag = true;
        start_time = ros::Time::now();
    }

}
void ApriltaginfoCallBack(const referee_msgs::Apriltag_info& msg)
{
    player_apriltag_pose[msg.tag_num].pose.position.x = msg.tag_pos_x;
    player_apriltag_pose[msg.tag_num].pose.position.y = msg.tag_pos_y;
    player_apriltag_pose[msg.tag_num].pose.position.z = msg.tag_pos_z;
}
void Score()
{
    ros::Time time1 =  ros::Time::now();
    for(int i=0;i<tags_num;i++)
    {
        if(player_apriltag_pose[i].pose.position.x!=0 || player_apriltag_pose[i].pose.position.y!=0 || player_apriltag_pose[i].pose.position.z!=0)
        {
            tag_flag[i]=true;
        }
    }
    for(int i=0;i<tags_num;i++)
    {
        if(tag_flag[i])
        {
            double x_err = abs(player_apriltag_pose[i].pose.position.x - real_apriltag_pose[i].pose.position.x);
            double y_err = abs(player_apriltag_pose[i].pose.position.y - real_apriltag_pose[i].pose.position.y);
            double z_err = abs(player_apriltag_pose[i].pose.position.z - real_apriltag_pose[i].pose.position.z);
            if(x_err < detect_error && y_err < detect_error&& z_err < detect_error)
            {
                tag_score[i]=1;
            }
        }
    }
    bool tag_check = all_of(tag_flag.begin(), tag_flag.end(), [](bool value){return value;});
    if(tag_check)
    {
        finish_detection = true;
    }
    if(start_flag)
    {
        int t=0;
        for(int i=0;i<tags_num;i++)
        {

            t+=tag_score[i];
        }
        total_score=t;
        cout<<"Total Score: "<<total_score<<endl;
        cout<<"Remain Time: "<<minutes<<" min "<<seconds<< " sec"<<endl;
        ros::Time time2 = ros::Time::now();
        double time_b = (time2 - time1).toSec();
        ros::Duration(1.0 - time_b).sleep();
        seconds--;
        if(seconds<0)
        {
            seconds = 59;
            minutes--;
        }
    }
}
void OdomInfo()
{
    gazebo_msgs::GetModelState uav;
    uav.request.model_name = "ardrone";
    uav.request.relative_entity_name = "world";
    if(client.call(uav))
    {
        model_pose.pose.position.x = uav.response.pose.position.x;
        model_pose.pose.position.y = uav.response.pose.position.y;
        model_pose.pose.position.z = uav.response.pose.position.z;
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "referee_system");
    ros::NodeHandle nh( "~" );
    rcv_start_flag = nh.subscribe("/start_flag", 10, Time_Count);
    rcv_tag_info = nh.subscribe("/apriltag_detection", 10, ApriltaginfoCallBack);
    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ROS_WARN("Referee system Load Sucessfully!");
    ros::Rate rate(100);
    while(ros::ok()&&(minutes>=0&&seconds>=0)&&!finish_detection)
    {
        OdomInfo();
        ApriltagInfo();
        Score();
        ros::spinOnce();
        rate.sleep();
    }
}