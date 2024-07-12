/*
Written by Jin Rhee 11th July 2024
*/

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <boost/histogram.hpp>
#include <boost/format.hpp>
#include <cassert>
#include <vector>

int poolNum;
int binNum;
int histogram_type;
float threshold;
float upper_bound;
float lower_bound;

bool plotted = false;
std::vector<float> pool(1);
int count = 0;

void getminmax(std::vector<float> arr, int arr_num, float minmax[])
{
    minmax[0] = *std::min_element(arr.begin(), arr.end());
    minmax[1] = *std::max_element(arr.begin(), arr.end());
}

void plot_histogram(std::vector<float> arr, int arr_num, int bin_num)
// Plain, boring histogram with auto min-max bounds (histogram_type=0)
{
    float minmax[2];
    getminmax(arr, arr_num, minmax);

    using namespace boost::histogram;
    auto h = make_histogram(axis::regular<>(bin_num, minmax[0], minmax[1]));
    for(int i=0; i<arr_num; i++){
        h(arr[i]);
    }
    for(auto&& x : indexed(h, coverage::all)){
        std::cout<<boost::format("bin %2i [%4.1f, %4.1f): %i\n")
          % x.index() % x.bin().lower() % x.bin().upper() % *x;
    }
    std::cout << std::flush;
}

void plot_histogram(std::vector<float> arr, int arr_num, int bin_num, float lb, float ub)
// Set lower and upper bound (histogram_type=1)
{
    using namespace boost::histogram;
    auto h = make_histogram(axis::regular<>(bin_num, lb, ub));
    for(int i=0; i<arr_num; i++){
        h(arr[i]);
    }
    for(auto&& x : indexed(h, coverage::all)){
        std::cout<<boost::format("bin %2i [%4.1f, %4.1f): %i\n")
          % x.index() % x.bin().lower() % x.bin().upper() % *x;
    }
    std::cout << std::flush;
}

void poolCallback(std_msgs::Float32 msg)
{
    if(count < poolNum){
        pool[count] = msg.data;
        count++;
        std::cout<<count<<' '<<poolNum<<std::endl;
    }
    else{
        if(!plotted){
            switch(histogram_type){
                case 0:
                    plot_histogram(pool, poolNum, binNum);
                    break;
                case 1:
                    plot_histogram(pool, poolNum, binNum, lower_bound, upper_bound);
                    break;
            }
            plotted = true;
        }
    }
}

template<typename T>
bool loadParam(ros::NodeHandle nh, const std::string& param_name, T& param_value, const T& default_value, const std::string& prefix="/speed_plot")
{
    if(nh.getParam(prefix+'/'+param_name, param_value)){
        std::cout<<"param '" << prefix << "/" << param_name << "' -> '" << param_value << "'" <<std::endl;
        return true;
    }
    else if(nh.getParam(ros::this_node::getName()+'/'+param_name, param_value)){
        std::cout<<"param '" << prefix << "/" << param_name << "' -> '" << param_value << "'"<<std::endl;
        return true;
    }
    param_value = default_value;
    std::cout<<"param '" << prefix << "/" << param_name << "' -> '" << param_value << "'"<< " (default)"<<std::endl;
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plothistogram");
    ros::NodeHandle nh;

    loadParam(nh, "poolNum", poolNum, 1000);
    loadParam(nh, "binNum", binNum, 50);
    loadParam(nh, "histogram_type", histogram_type, 1);
    loadParam(nh, "threshold", threshold, 0.5f);
    loadParam(nh, "upper_bound", upper_bound, 7.2f);
    loadParam(nh, "lower_bound", lower_bound, 0.0f);

    // Resize vector
    pool.resize(poolNum);

    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("/dynamic/vels", 1000, poolCallback);
    ros::spin();
    return 0;
}