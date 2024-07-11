/*
Written by Jin Rhee 11th July 2024
*/

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <boost/histogram.hpp>
#include <cassert>

const int poolNum = 1000;
const int binNum = 10;
const int threshold = 0.5;

using namespace std;

void plot(float nums[])
{
    cout << *std::min_element(nums, nums+poolNum) << ' ' << *std::max_element(nums, nums+poolNum) << endl;
}

void poolCallback(std_msgs::Float32 msg)
{
    float nums[poolNum];
    for(int i=0; i<poolNum; i++){
        nums[i] = msg.data;
    }
    plot(nums);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plothistogram");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("/dynamic/vels", 1000, poolCallback);
    ros::spin();
    return 0;
}