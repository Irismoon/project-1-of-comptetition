#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include <iostream>

#include "ardrone_autonomy/Navdata.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

std_msgs::Empty order;
geometry_msgs::Twist cmd;

const float vel_kp=0.00,vel_ki=0.00,vel_kd=0.00;
const float pos_kp=0.00,pos_ki=0.00,pos_kd=0.00;
#define LOOP_RATE 10

struct raw_state
{
    Vector3f  pos_b;
    Vector3f pos_f;
    Vector3f vel_b;
    Vector3f vel_f;
};
struct output
{
    Vector3f vel_sp;
};
struct control
{
    Vector3f vel_sp;
    Vector3f pos_sp;
};

raw_state raw_stat;
output out;
control contro;

void odometryCallback(const nav_msgs::Odometry &msg){
    raw_stat.pos_b(0)= msg.pose.pose.position.x;//unit: m
    raw_stat.pos_b(1)= msg.pose.pose.position.y;
    raw_stat.vel_b(0)= msg.twist.twist.linear.x;//unit:m/s
    raw_stat.vel_b(1)= msg.twist.twist.linear.y;
}

void navCallback(const ardrone_autonomy::Navdata &msg)
{
    static bool start=true;
    static float last_time = 0;
    if(start){
        start = false;
        last_time = msg.tm;
    }

    float dt = (msg.tm - last_time)/1000000.0;
    last_time = msg.tm;
//    raw_stat.vel_b(0) = msg.vx/1000;
//    raw_stat.vel_b(1) = msg.vy/1000;
//    raw_stat.pos_b = raw_stat.vel_b * dt;
}

void pid_pos(Vector3f& actual,Vector3f& set,Vector3f& control)
{
    static Vector3f err_last;
    static Vector3f err_int;
    static bool new_start = true;
    Vector3f err_pos;
    Vector3f err_d;
    err_pos = set - actual;
    if (new_start)
    {
        err_last = err_pos;
        err_int = Vector3f::Zero();
        new_start = false;
    }
    err_d = (err_pos - err_last)*LOOP_RATE;
    control = err_pos * pos_kp + err_d * pos_kd + err_int * pos_ki;
    err_last = err_pos;
    err_int += err_pos / LOOP_RATE;
}
void pid_vel(Vector3f& actual,Vector3f& set,Vector3f& control)
{
    static Vector3f err_lastt;
    static Vector3f err_intt;
    static bool new_startt = true;
    Vector3f err_vel;
    Vector3f err_d;
    err_vel = set - actual;
    if (new_startt)
    {
        err_lastt = err_vel;
        err_intt = Vector3f::Zero();
        new_startt = false;
    }
    err_d = (err_vel - err_lastt)*LOOP_RATE;
    control = err_vel * vel_kp + err_d * vel_kd + err_intt * vel_ki;
    err_lastt = err_vel;
    err_intt += err_vel / LOOP_RATE;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "circlecontrol");
    ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Subscriber sub = n.subscribe("/ardrone/odometry",1,odometryCallback);
   ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata", 1, navCallback);
    ros::Rate loop_rate(LOOP_RATE);
    int index = 0;

    raw_stat.pos_b = Vector3f::Zero();
    raw_stat.pos_f = Vector3f::Zero();
    raw_stat.vel_b = Vector3f::Zero();
    raw_stat.vel_f = Vector3f::Zero();
    out.vel_sp = Vector3f::Zero();;
    contro.pos_sp = Vector3f::Zero();
    contro.vel_sp = Vector3f::Zero();


    while(ros::ok())
    {
        index = index+1;
	ROS_INFO("index:%d",index);
	/*if (index<=50)
        {
            raw_stat.vel_f(0) = index/1000.0;
            raw_stat.pos_f(0) = index/1000.0 * index /2/ LOOP_RATE;
        }else if((index>300)&&(index<=500))
        {
            raw_stat.vel_f(0) = -0.001*index+1;
            raw_stat.pos_f(0) = (600+800)*0.2/2/LOOP_RATE+ (-0.001*index+0.1+0.02)*(index-800)/2/LOOP_RATE;
        }else if((index<=50)&&(index>0))
        {
            raw_stat.pos_f(0) = 200*0.02/2/LOOP_RATE+(index-200)*0.02/LOOP_RATE;
        }else
        {
            raw_stat.vel_f(0) = 0.0;
            raw_stat.pos_f(0) = (600+1000)*0.02/2/LOOP_RATE;
        }
        //raw_stat.vel_f(0) = raw_stat.vel_f(0)  * 10;
	ROS_INFO("vset:%f",raw_stat.vel_f(0));
        //raw_stat.pos_f(0) = raw_stat.pos_f(0)  * 10;
	*/
	
        pid_pos(raw_stat.pos_b,raw_stat.pos_f,contro.pos_sp);
	//ROS_INFO("px_act:%f,py_act:%f,px_set:%f,py_set:%f",raw_stat.pos_b(0),raw_stat.pos_b(1),raw_stat.pos_f(0),raw_stat.pos_f(1));
	ROS_INFO("posafter_pid:x%fy%f",contro.pos_sp(0),contro.pos_sp(1));
        //out.vel_sp = raw_stat.vel_f+contro.pos_sp;
        pid_vel(raw_stat.vel_b,out.vel_sp,contro.vel_sp);
	//ROS_INFO("Vel,act,x:%f,y:%f,set,x:%f,y:%f",raw_stat.vel_b(0),raw_stat.vel_b(1),out.vel_sp(0),out.vel_sp(1));
	//ROS_INFO("finalvel,x:%f,y:%f",contro.vel_sp(0),contro.vel_sp(1));
        
        cmd.linear.x = contro.vel_sp(0)/10.0;
        cmd.linear.y = contro.vel_sp(1)/10.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        //cmd.linear.x = 0.02;
        //cmd.linear.y = 0.005;

        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
