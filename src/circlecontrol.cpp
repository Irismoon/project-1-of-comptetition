#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>

std_msgs::Empty order;
geometry_msgs::Twist cmd;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circlecontrol");
    ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Rate loop_rate(10);
    int index = 0;double x = 0;double y = 0;double setx = 0.03;

    while(ros::ok())
    {
        index = index+1;
	x = 3*pow(index/10.0,0.5)/1000.0;
	//x = sin(index)
        cmd.linear.x = (x > setx)?setx:x;
        //printf('y%f',cmd.linear.y);
        y = setx-x;//sqrt(pow(setx,2)-pow(x,2));//(5.0/index>0.04) ? 0.04 : 5.0/index;
	cmd.linear.y = y;
        //printf('x%f',cmd.linear.x);
	ROS_INFO("x:%f;y:%f",x,y);
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
