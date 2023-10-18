#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>

std::ofstream ofs_("/home/han/Documents/odom_to_csv.csv");


void cb(nav_msgs::Odometry::ConstPtr msg)
{
    ofs_ << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_csv");

    ros::NodeHandle nh;

    ofs_ << "x, y" << std::endl;

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odometry/gps", 10, &cb);

    ros::spin();

    ofs_.close();

    return 0;
}