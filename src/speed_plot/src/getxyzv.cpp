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
#include <bitset>

using namespace std;

struct AevaPointXYZIRT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    float reflectivity;
    float velocity;
    int32_t time_offset_ns;
    uint8_t line_index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (AevaPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, reflectivity, reflectivity) (float, velocity, velocity) 
    (std::int32_t, time_offset_ns, time_offset_ns) (std::uint8_t, line_index, line_index)
)

ros::Publisher publish_moving;

float bintofloat(int num, const unsigned char* ints)
{
    ostringstream stream;
    string s;
    uint32_t temp_val;
    float f;

    for(int i=num-1; i>=0; i--){
        //cout << i << endl;
        //cout << (int)(ints[i]) << ' ' << bitset<8>((uint8_t)(ints[i])).to_string() << ' ';
        stream << bitset<8>((uint8_t)(ints[i])).to_string();        // Convert each 8-bit int to bitstring
    }

    s = stream.str();                                               // Get string stream as string
    std::bitset<32> temp (s);                                       // Set bitfield with bitstring

    temp_val = temp.to_ulong();                                     // Read as long int
    assert(sizeof(temp_val) == sizeof(f));
    memcpy(&f, &temp_val, sizeof(f));                               // Copy memory and read as float
    
    return f;
}

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

    pcl::PointCloud<AevaPointXYZIRT> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    std::cout<<"velocity : "<<pl_orig.points[0].velocity<<std::endl;


    int _width = msg->width;                         // Width of data array
    int _height = msg->height;                       // Height of data array
    int _point_step = msg->point_step;               // Length of one point in bytes (= number of entries?)
    int _row_step = msg->row_step;                   // Length of one row in bytes
    bool _is_dense = msg->is_dense;                   // True if there are no invalid points

    float nsec;
    std::vector<sensor_msgs::PointField_<std::allocator<void> > > myvector (4);
    int x_offset;
    int y_offset;
    int z_offset;
    int vel_offset;

    /*
    x: FLOAT32
    y: FLOAT32
    z: FLOAT32
    intensity: FLOAT32
    reflectivity: FLOAT32
    velocity: FLOAT32
    time_offset_ns: INT32
    line_index: UINT8
    */

    // Print out time of scan
    //cout << msg->header.stamp.sec << '.' << msg->header.stamp.nsec << endl;
    nsec = msg->header.stamp.nsec;
    
    // Get offsets
    myvector = msg->fields;
    x_offset = myvector[0].offset;
    y_offset = myvector[1].offset;
    z_offset = myvector[2].offset;
    vel_offset = myvector[4].offset;

    // Allocate data arrays
    float x[_width];
    float y[_width];
    float z[_width];
    float vel[_width];

    // For each row (scan)
    for(int j = 0; j <_width; j++){
        // Convert to floats
        x[j] = bintofloat(4, &(msg->data[j*_point_step + x_offset]));
        y[j] = bintofloat(4, &(msg->data[j*_point_step + y_offset]));
        z[j] = bintofloat(4, &(msg->data[j*_point_step + z_offset]));
        vel[j] = bintofloat(4, &(msg->data[j*_point_step + vel_offset]));
        pubfloat32(vel[j]);
        //cout << x[j] << ' ' << y[j] << ' ' << z[j] << ' ' << vel[j] << endl;

        if(msg->header.stamp.nsec != nsec){
            cout << "!!!" << msg->header.stamp.sec << '.' << msg->header.stamp.nsec << endl;
        }
    }
    // Find min and max vels for each row
    coutminmax(x, _width);
    coutminmax(y, _width);
    coutminmax(z, _width);
    coutminmax(vel, _width);
    cout << endl;
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