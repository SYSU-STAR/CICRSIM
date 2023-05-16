#include <ros/ros.h>
#include <referee_msgs/Apriltag_info.h>
ros::Publisher topic0;

void Pub0()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =0;
    msg.tag_pos_x = 5.05;
    msg.tag_pos_y = 1.85;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub1()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =1;
    msg.tag_pos_x = -0.62;
    msg.tag_pos_y = 3.23;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub2()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =2;
    msg.tag_pos_x = -4.5;
    msg.tag_pos_y = -1.52;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub3()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =3;
    msg.tag_pos_x = -3.98;
    msg.tag_pos_y = -1.30;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub4()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =4;
    msg.tag_pos_x = 5.52;
    msg.tag_pos_y = -3.00;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub5()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =5;
    msg.tag_pos_x = -6.56;
    msg.tag_pos_y = -3.05;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub6()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =6;
    msg.tag_pos_x = -9.27;
    msg.tag_pos_y = -0.2;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}
void Pub7()
{
    referee_msgs::Apriltag_info msg;
    msg.tag_num =7;
    msg.tag_pos_x = -9.5;
    msg.tag_pos_y = 3.84;
    msg.tag_pos_z = 1.2;
    topic0.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_sim");
    ros::NodeHandle nh;
    topic0 = nh.advertise<referee_msgs::Apriltag_info>("/apriltag_detection",10);
    // referee_msgs::Apriltag_info msg;
    // msg.tag_pos_x = 5.05;
    // msg.tag_pos_y = 2.82;
    // msg.tag_pos_z = 1.2;
    // for(int i=0;i<8;i++)
    // {
    //     ros::Duration(3).sleep();
    //     msg.tag_num = i;    
    //     topic0.publish(msg);

    // }
    ros::Duration(3).sleep();
    Pub0();
    ros::Duration(3).sleep();
    Pub1();
    ros::Duration(3).sleep();
    Pub2();
    ros::Duration(3).sleep();
    Pub3();
    ros::Duration(3).sleep();
    Pub4();
    ros::Duration(3).sleep();
    Pub5();
    ros::Duration(3).sleep();
    Pub6();
    ros::Duration(3).sleep();
    Pub7();
}