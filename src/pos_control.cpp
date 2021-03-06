#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>

#include "ardrone_autonomy/Navdata.h"
#include "position_estimate/renew.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

std_msgs::Empty order;
geometry_msgs::Twist cmd;
int current_index;
const float pos_kp=0.2,pos_ki=0.0/*0.03*/,pos_kd=0.0/*0.01*/,yaw_kp=0.5;
#define LOOP_RATE 10
#define row_num 292
#define col_num 4
#define key_point_num 3
#define measured_pos "/home/wade/catkin_ws/src/project-1-of-comptetition/src/setpoint.csv"

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
struct yaw_control
{
    float yaw_set;
    float yaw_actual;
};
raw_state raw_stat;
output out;
control contro;
yaw_control yaw_contro;
float out_yaw;
bool variate = true;
float start_x=0.0;
float start_y=0.0;

void odometryCallback(const geometry_msgs::Point &msg){
    raw_stat.pos_b(0)= msg.x;//unit: m ?????? what the hell?????
    raw_stat.pos_b(1)= msg.y;
    static bool record=true;
    if(record)
    {
        start_x = raw_stat.pos_b(0);
        start_y = raw_stat.pos_b(1);
        record = false;
    }

    raw_stat.pos_b(0) = raw_stat.pos_b(0) - start_x;
    raw_stat.pos_b(1) = raw_stat.pos_b(1) - start_y;
}
void yawCallback(const std_msgs::Float32 &msg)
{
    static bool first=true;
    static float yaw_ini;
    
    if(first)
    {  yaw_ini = msg.data;
       first = false;
    }

    yaw_contro.yaw_actual = msg.data - yaw_ini;
}
bool read_csv(char *filepath, MatrixXf &setpoint)
{
    string pixel;
    ifstream file(filepath,ifstream::in);
    if(!file)
    {
        cout << "csv read fail" << endl;
        return false;
    }
    int nc = setpoint.cols()*setpoint.rows();
    int eolElem = setpoint.cols() - 1;
    int elemCount = 0;
    for(int i = 0;i<nc;i++)
    {
        if(elemCount==eolElem)
        {
            getline(file,pixel,'\n');
            setpoint(int(i/col_num),elemCount)=(float)atof(pixel.c_str());
            elemCount = 0;
        }
        else{
            getline(file,pixel,',');
            setpoint(int(i/col_num),elemCount)=(float)atof(pixel.c_str());
            elemCount++;
        }
    }

    return true;
}
void Is_RenewCallback(const position_estimate::renew &msg)
{
    variate = msg.isRenew;
    if (variate)
        current_index = msg.index;
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
void pid_yaw(float yaw_actual,float yaw_set,float &out_yaw)
{

    out_yaw = yaw_kp * (yaw_set - yaw_actual);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_control");
    ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_ref", 1);
    ros::Publisher index_pub = n.advertise<std_msgs::Int8>("index",1);
    ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Subscriber odo_sub = n.subscribe("/ardrone_position",1,odometryCallback);
    ros::Subscriber flag_sub = n.subscribe("is_renew",1,Is_RenewCallback);
    ros::Subscriber yaw_sub = n.subscribe("/ardrone/yaw",1,yawCallback);
    ros::Rate loop_rate(LOOP_RATE);

    raw_stat.pos_b = Vector3f::Zero();
    raw_stat.pos_f = Vector3f::Zero();
    raw_stat.vel_b = Vector3f::Zero();
    raw_stat.vel_f = Vector3f::Zero();
    out.vel_sp = Vector3f::Zero();;
    contro.pos_sp = Vector3f::Zero();
    contro.vel_sp = Vector3f::Zero();
    yaw_contro.yaw_actual = 0.0;
    yaw_contro.yaw_set = 0.0;
    MatrixXf setpoint(row_num,col_num);
    Vector3d key_index(98,194,292);
    read_csv(measured_pos,setpoint);
    current_index = 0;

    while(ros::ok())
    {
        ROS_INFO("index:%d",current_index);
        ROS_INFO("vset:x=%f y=%f",raw_stat.vel_f(0),raw_stat.vel_f(1));
        ROS_INFO("pset:x=%f y=%f",raw_stat.pos_f(0),raw_stat.pos_f(1));

        if(!variate)
        {
            raw_stat.pos_f(0) = setpoint(current_index,2);
            raw_stat.pos_f(1) = setpoint(current_index,3);
            raw_stat.vel_f(0) = setpoint(current_index,0);
            raw_stat.vel_f(1) = setpoint(current_index,1);
            current_index++;
            std_msgs::Int8 tmp;
            tmp.data = current_index;
            index_pub.publish(tmp);
        }

        for(int i=0;i<key_point_num;i++)
        {
            if(key_index(i)==current_index)
                variate = true;
        }


        pid_pos(raw_stat.pos_b,raw_stat.pos_f,contro.pos_sp);
        out.vel_sp = raw_stat.vel_f+contro.pos_sp;
        pid_yaw(yaw_contro.yaw_actual,yaw_contro.yaw_set,out_yaw);

        ROS_INFO("vel_sp:x=%f y=%f",out.vel_sp(0),out.vel_sp(1));
        ROS_INFO("position:x=%f y=%f",raw_stat.pos_b(0),raw_stat.pos_b(1));

        cmd.linear.x = out.vel_sp(0);
        cmd.linear.y = out.vel_sp(1);
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = -out_yaw;

        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
        if (current_index>=row_num)
            break;
    }
    return 0;
}
