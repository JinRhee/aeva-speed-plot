#ifndef GETXYZV_H
#define GETXYZV_H

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
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


float bintofloat(int num, const unsigned char* ints)
// Does what it's supposed to do, but there seems to be a semantic error somewhere. I give up for now
{
    const int size = sizeof(unsigned char) * 8;             // Get size of unsigned char (bits)
    unsigned char temp[num];                                // Allocate non-const memory for unsigned chars
    float float_ptr[1];                                     // Declare pointer to converted float value

    /*
    for(int i=0; i<num; i++){
        std::cout<<(int)(ints[i])<<' ';
    }
    std::cout<<std::endl;
    */

    memcpy(float_ptr, ints, sizeof(float));

    return *float_ptr;
}

float bintofloatLegacy(int num, const unsigned char* ints)
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
#endif