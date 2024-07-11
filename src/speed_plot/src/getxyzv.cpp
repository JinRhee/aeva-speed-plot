/*
* Written by Jin Rhee 8th July 2024
*/

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include "speed_plot/getxyzv.hpp"

#define TOGGLE_COUT false

ros::Publisher publish_moving;

void pubfloat32(float f)
{
    std_msgs::Float32 msg;
    msg.data = f;
    publish_moving.publish(msg);
}

void coutminmax(float nums[], int n)
{
    cout << *std::min_element(nums, nums+n) << ' ' << *std::max_element(nums, nums+n);
    float diff = *std::max_element(nums, nums+n) - *std::min_element(nums, nums+n);
    cout << ' ' << diff;
    if (diff == 0){
        cout << "                ====" << endl;
    }
    else{
        cout << endl;
    }
}

void readCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    int _width = msg->width;

    pcl::PointCloud<AevaPointXYZIRT> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    
    // Collate x,y,z,vels
    float x[_width];
    float y[_width];
    float z[_width];
    float vel[_width];

    for(int i=0; i<_width; i++){
        x[i] = pl_orig.points[i].x;
        y[i] = pl_orig.points[i].y;
        z[i] = pl_orig.points[i].z;
        vel[i] = pl_orig.points[i].velocity;
        pubfloat32(vel[i]);
    }

    if(TOGGLE_COUT){
        // Find min and max vels for each row
        coutminmax(x, _width);
        coutminmax(y, _width);
        coutminmax(z, _width);
        coutminmax(vel, _width);
        cout << endl;    
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "getxyzv");
    ros::NodeHandle nh;
    publish_moving = nh.advertise<std_msgs::Float32>("dynamic/vels", 1000);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("aeva/points", 5000, readCallback);

    ros::spin();
    return 0;
}