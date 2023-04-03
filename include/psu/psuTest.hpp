#ifndef PSUTEST
#define PSUTEST
#include <memory>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <DxkdpLib/DxkdpLib.hpp>
#include "ros_coils/magField.h"
#include "ros_coils/VI.h"
#include "ros_coils/Polarity.h"
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

class PsuROSWrapper
{
private:
    // Unique ptrs here
    std::unique_ptr<DXKDP_PSU> X1_; // smart pointer to psu instance for the node. 

    // Publishers here

    // Publisher timers here

    // Publisher frequency here

    // Subscribers here
    // ros::Subscriber currentSubscriber_;
    // ros::Subscriber voltageSubscriber_;
    ros::Subscriber viSubscriber_; // subscriber to VI topic. Updates class values and calls funcs from X1.
    ros::Subscriber polaritySubscriber_; // subscriber to polarity topic. Updates class values and calls funcs from X1.

    // Services here
    ros::ServiceServer powerOnServer_;
    ros::ServiceServer shutdownServer_;

    int currentI, currentV; // internal values for cached I and V.
    float vConv, iConv; // minimum psu increment for voltage and current.
    uint8_t currentPolarity; // currently stored polarity
    std::string nodeName; // node name. Equal to PSU name.

public:
    /**
     * @brief Construct a new Psu ROS Wrapper object
     *
     * @param nh ros node handle
     */
    PsuROSWrapper(ros::NodeHandle *nh);
    /**
     * @brief callback Function for current write. Unused. Look at VIWrite instead.
     *
     * @param msg std/Float32. Current to write
     */
    void callbackCurrentWrite(const std_msgs::Float32 &msg);

    /**
     * @brief callback Function for Voltage write. Unused. Look at VIWrite instead.
     *
     * @param msg std/Float32. Voltage to write
     */
    void callbackVoltageWrite(const std_msgs::Float32 &msg);
    /**
     * @brief callback Function for VI write. 
     * 
     * @param msg VI. custom msg containing voltage and current values.
     */
    void callbackVIWrite(const ros_coils::VI &msg);
    /**
     * @brief Callback for polarity setting.
     * 
     * @param msg Polarity. custom msg containing polarity values.
     */
    void callbackPolarity(const ros_coils::Polarity &msg);
    bool callbackSetup(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackShutdown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

bool compare_float(float x, float y, float epsilon = 0.01f)
{
    // std::cout << "x: " << x << "- y: " << y << "= " << fabs(x - y) << "\n";
    if (fabs(x - y) < epsilon)
    {
        return true;
    } // they are same}
    {
        return false;
    } // they are not same}
}
#endif