#include <ros/ros.h>
#include <boost/format.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/SwarmCommand.h"
//#include "prometheus_msgs/Case3Result.h"
//#include "prometheus_msgs/StationCommandCase3.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "uav_utils/geometry_utils.h"
#include "printf_utils.h"

// 集群数量
int swarm_num;     
// 无人机名字                             
string uav_name;  
// 无人机编号                         
int uav_id; 
int control_flag;
// ego输出
quadrotor_msgs::PositionCommand ego_traj_cmd;
geometry_msgs::PoseStamped ego_date;      //using rviz
geometry_msgs::Pose q;
bool get_ego_traj;
// 发布的控制指令
prometheus_msgs::SwarmCommand Command_Now;

ros::Subscriber ego_ouput_sub;
ros::Publisher command_pub;
ros::Publisher ego_pos_rviz_ref_pub;

geometry_msgs::Pose to_quaternion(double pitch, double roll, double yaw)
{

    double t0, t1, t2, t3, t4, t5;
    t0 = cos(yaw * 0.5);
    t1 = sin(yaw * 0.5);
    t2 = cos(roll * 0.5);
    t3 = sin(roll * 0.5);
    t4 = cos(pitch * 0.5);
    t5 = sin(pitch * 0.5);
    q.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;

}
void ego_ouput_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    ego_traj_cmd = *msg;
    get_ego_traj = true;


    ego_date.header = ego_traj_cmd.header;
    ego_date.pose.position.x = ego_traj_cmd.position.x;
    ego_date.pose.position.y = ego_traj_cmd.position.y;
    ego_date.pose.position.z = ego_traj_cmd.position.z;
    q = to_quaternion(0,0,ego_traj_cmd.yaw);
    ego_date.pose.orientation.x = q.orientation.x;
    ego_date.pose.orientation.y = q.orientation.y;
    ego_date.pose.orientation.z = q.orientation.z;
    ego_date.pose.orientation.w = q.orientation.w;
    //cout << YELLOW <<  " Arrive the goal " << TAIL <<endl;

    ego_pos_rviz_ref_pub.publish(ego_date);

    if(ego_traj_cmd.velocity.x == 0)
    {
        get_ego_traj = false;
        //cout << YELLOW <<  " Arrive the goal " << TAIL <<endl;
    }else
    {
        Command_Now.header.stamp     = ros::Time::now();
        Command_Now.Mode             = prometheus_msgs::SwarmCommand::Move;
        Command_Now.Command_ID       = Command_Now.Command_ID + 1;
        Command_Now.source           = uav_name;

        if(control_flag == 0)
        {
            // pos_controller
            Command_Now.Move_mode          = prometheus_msgs::SwarmCommand::TRAJECTORY;
        }
        else if (control_flag ==1)
        {
            // px4_sender
            Command_Now.Move_mode          = prometheus_msgs::SwarmCommand::XYZ_POS;
        }
        else if (control_flag ==2)
        {
            // px4_sender
            Command_Now.Move_mode          = prometheus_msgs::SwarmCommand::XY_VEL_Z_POS;
        }
        Command_Now.position_ref[0]     = ego_traj_cmd.position.x;
        Command_Now.position_ref[1]     = ego_traj_cmd.position.y;
        Command_Now.position_ref[2]     = ego_traj_cmd.position.z;
        Command_Now.velocity_ref[0]     = ego_traj_cmd.velocity.x;
        Command_Now.velocity_ref[1]     = ego_traj_cmd.velocity.y;
        Command_Now.velocity_ref[2]     = ego_traj_cmd.velocity.z;
        Command_Now.acceleration_ref[0]     = ego_traj_cmd.acceleration.x;
        Command_Now.acceleration_ref[1]     = ego_traj_cmd.acceleration.y;
        Command_Now.acceleration_ref[2]     = ego_traj_cmd.acceleration.z;
        Command_Now.yaw_ref = uav_utils::normalize_angle(ego_traj_cmd.yaw);


        // Command_Now.yaw_rate_ref         = ego_traj_cmd.yaw_dot;
        
        command_pub.publish(Command_Now);   
    }
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_traj_to_cmd");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
    
    // 集群数量
    nh.param("swarm_num", swarm_num, 1);
    // 无人机编号 1号无人机则为1
    nh.param("uav_id", uav_id, 0);
    nh.param("control_flag", control_flag, 0);

    uav_name = "/uav" + std::to_string(uav_id);

    // 【订阅】EGO的轨迹输出(traj_server的输出)
    ego_ouput_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(uav_name + "/prometheus/ego/traj_cmd", 1, ego_ouput_cb);

    // 【发布】 路径指令 （发送至swarm_controller.cpp）
    command_pub = nh.advertise<prometheus_msgs::SwarmCommand>(uav_name + "/prometheus/swarm_command", 1);

    ego_pos_rviz_ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/ego_pos_ref", 1);
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}

