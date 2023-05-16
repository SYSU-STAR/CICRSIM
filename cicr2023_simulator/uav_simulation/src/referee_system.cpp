#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include "referee_msgs/Apriltag_info.h"
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;
ros::Subscriber rcv_start_flag, rcv_tag_info;
ros::ServiceClient client;
int tags_num = 8;
int minutes = 3;
int seconds = 0; 
int total_score;
bool start_flag = false;
double detect_error = 0.03;//允许的3cm检测误差
bool finish_detection = false;//如果全部二维码被发现，视为完成探索
vector<geometry_msgs::PoseStamped>real_apriltag_pose(tags_num);//仿真场景中的二维码位置
vector<geometry_msgs::PoseStamped>player_apriltag_pose(tags_num);//接收参赛选手发布的二维码位置
vector<bool>tag_flag = {false,false,false,false,false,false,false,false};//所有二维码默认没有检测到，置为false
vector<int>tag_score = {0,0,0,0,0,0,0,0};
void ApriltagInfo()//获取二维码位置
{
    vector<gazebo_msgs::GetModelState> apriltag(tags_num);
    vector<string> apriltag_names = {"static_apriltag_0","static_apriltag_1","static_apriltag_2","static_apriltag_3","static_apriltag_4","static_apriltag_5","static_apriltag_6","static_apriltag_7"};
    for(int i=0;i<tags_num;i++)
    {
        apriltag[i].request.model_name= apriltag_names[i];
        apriltag[i].request.relative_entity_name = "world";
        // ROS_INFO("Tag_Name: %s Pos_x: %d Pos_y: %d Pos_z: %d",apriltag[i].request.model_name,apriltag[i].response.pose.position.x,apriltag[i].response.pose.position.y,apriltag[i].response.pose.position.z );
        if(client.call(apriltag[i]))
        {
            real_apriltag_pose[i].pose.position.x = apriltag[i].response.pose.position.x;
            real_apriltag_pose[i].pose.position.y = apriltag[i].response.pose.position.y;
            real_apriltag_pose[i].pose.position.z = apriltag[i].response.pose.position.z;
        }
    }
}
void Time_Count(const geometry_msgs::PoseStamped& msg)//回调函数，2d nav_goal 触发计时开始
{
    start_flag = true;
}
void ApriltaginfoCallBack(const referee_msgs::Apriltag_info& msg)//回调函数，订阅参赛选手发布的二维码位置
{
    player_apriltag_pose[msg.tag_num].pose.position.x = msg.tag_pos_x;
    player_apriltag_pose[msg.tag_num].pose.position.y = msg.tag_pos_y;
    player_apriltag_pose[msg.tag_num].pose.position.z = msg.tag_pos_z;
}
void Score()//评分函数
{
    for(int i=0;i<tags_num;i++)//判断是否检测到了二维码
    {
        if(player_apriltag_pose[i].pose.position.x!=0&&player_apriltag_pose[i].pose.position.y!=0&&player_apriltag_pose[i].pose.position.z!=0)
        {
            // cout<<"TAG "<<i<<" has been detected"<<endl;
            tag_flag[i]=true;
            // cout<<"_____________________________________________________"<<endl;
        }
    }
    for(int i=0;i<tags_num;i++)
    {
        if(tag_flag[i])//如果检测到，进行精确度判断
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
        // cout<<"successfully detect all tags"<<endl;
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
        this_thread::sleep_for(chrono::seconds(1));
        seconds--;
        if(seconds<0)
        {
            seconds = 59;
            minutes--;
        }
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "referee_system");
    ros::NodeHandle nh;
    rcv_start_flag = nh.subscribe("/move_base_simple/goal", 10, Time_Count);
    rcv_tag_info = nh.subscribe("/apriltag_detection", 10, ApriltaginfoCallBack);
    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Rate rate(200);
    while(ros::ok()&&(minutes>=0&&seconds>=0)&&!finish_detection)
    {
        ApriltagInfo();
        Score();
        ros::spinOnce();
        rate.sleep();
    }

}