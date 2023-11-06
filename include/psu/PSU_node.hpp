#ifndef PSUNODE
#define PSUNODE
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include <DxkdpLib/DxkdpLib.hpp>
#include <memory>
#include <sstream>
#include <vector>

#include "ros_coils/Polarity.h"
#include "ros_coils/VI.h"
#include "ros_coils/magField.h"

class PSU_node {
   private:
    std::unique_ptr<DXKDP_PSU>
        PSU;  // smart pointer to the DXKDPLIB instance used by the node.
    ros::Subscriber VI_sub_;
    ros::ServiceServer powerOnServer_;
    ros::ServiceServer shutdownServer_;

    int currentI, currentV;   // internal values for cached I and V.
    float vConv, iConv;       // minimum psu increment for voltage and current.
    uint8_t currentPolarity;  // currently stored polarity
    std::string nodeName;     // node name. Equal to PSU name.

    bool debugMode = true;  // debug flag. If true, PSU is never instantiated
                            // and function calls are empty
   public:
    /**
     * @brief Construct a new psu node object, initialize PSU and ROS.
     *
     * @param nh
     */
    PSU_node(ros::NodeHandle *nh);

    /**
     * @brief Write VI. Uses nodeName to discern between Gen1 and Gen2 supplies.
     *
     * @param msg contains signed voltage and current values.
     */
    void callbackVIWrite(const ros_coils::VI &msg);

    bool callbackSetup(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res);
    bool callbackShutdown(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res);
};

bool compare_float(float x, float y, float epsilon = 0.01f) {
    // std::cout << "x: " << x << "- y: " << y << "= " << fabs(x - y) << "\n";
    if (fabs(x - y) < epsilon) {
        return true;
    }                  // they are same}
    { return false; }  // they are not same}
}

#endif