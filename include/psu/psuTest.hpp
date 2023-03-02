#ifndef PSUTEST
#define PSUTEST
#include <memory>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <DxkdpLib/DxkdpLib.hpp>
#include "ros_coils/magField.h"
#include <std_msgs/Int32.h>
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

    // Services here
    ros::ServiceServer powerOnServer_;
    ros::ServiceServer shutdownServer_;

    int currentI, currentV;

public:
    PsuROSWrapper(ros::NodeHandle *nh);
    void callbackCurrentWrite(const std_msgs::Int32 &msg);
    void callbackVoltageWrite(const std_msgs::Int32 &msg);
    void callbackSetup(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void callbackShutdown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};
#endif