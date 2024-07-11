/*
Written by Jin Rhee 11th July 2024
*/

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <boost/histogram.hpp>
#include <boost/format.hpp>
#include <cassert>

const int poolNum = 1e5;
const int binNum = 50;
const int threshold = 0.5;

bool plotted = false;
float pool[poolNum];
int count = 0;
int h_type = 1;
float bound_edges = {};
float upper_bound = 7.2;
float lower_bound = 0.0;

void getminmax(float arr[], int arr_num, float minmax[])
{
    minmax[0] = *std::min_element(arr, arr+arr_num);
    minmax[1] = *std::max_element(arr, arr+arr_num);
}

void plot_histogram(float arr[], int arr_num, int bin_num)
// Plain, boring histogram with auto min-max bounds (h_type=0)
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

void plot_histogram(float arr[], int arr_num, int bin_num, float lb, float ub)
// Set lower and upper bound (h_type=1)
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
/*
void plot_histogram(float arr[], int arr_num, int bin_num, float edges[])
// Set edges with array (h_type=2)
{
    using namespace boost::histogram;
    auto h = make_histogram(axis::variable<>(edges));
    for(int i=0; i<arr_num; i++){
        h(arr[i]);
    }
    for(auto&& x : indexed(h, coverage::all)){
        std::cout<<boost::format("bin %2i [%4.1f, %4.1f): %i\n")
          % x.index() % x.bin().lower() % x.bin().upper() % *x;
    }
    std::cout << std::flush;
}
*/
void poolCallback(std_msgs::Float32 msg)
{
    if(count < poolNum){
        pool[count] = msg.data;
        count++;
        std::cout<<count<<' '<<poolNum<<std::endl;
    }
    else{
        if(!plotted){
            switch(h_type){
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plothistogram");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("/dynamic/vels", 1000, poolCallback);
    ros::spin();
    return 0;
}