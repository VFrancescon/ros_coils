#ifndef PSUTEST
#define PSUTEST
#include <memory>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <DxkdpLib/DxkdpLib.hpp>
#include "ros_coils/magField.h"
#include "ros_coils/VI.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

class PsuROSWrapper
{
private:
    // Unique ptrs here
    std::unique_ptr<DXKDP_PSU> X1_;

    // Publishers here

    // Publisher timers here

    // Publisher frequency here

    // Subscribers here
    ros::Subscriber currentSubscriber_;
    ros::Subscriber voltageSubscriber_;
    ros::Subscriber viSubscriber_;

    // Services here
    ros::ServiceServer powerOnServer_;
    ros::ServiceServer shutdownServer_;

    int currentI, currentV;
    float vConv, iConv;

public:
    PsuROSWrapper(ros::NodeHandle *nh);
    void callbackCurrentWrite(const std_msgs::Float32 &msg);
    void callbackVoltageWrite(const std_msgs::Float32 &msg);
    void callbackVIWrite(const ros_coils::VI &msg);
    bool callbackSetup(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackShutdown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

bool compare_float(float x, float y, float epsilon = 0.01f)
{   
    std::cout << "x: " << x << "- y: " << y << "= " << fabs(x - y) << "\n"; 
    if (fabs(x - y) < epsilon)
    {
        return true;
    } // they are same}
    {
        return false;
    } // they are not same}
}
#endif