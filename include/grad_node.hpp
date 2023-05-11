#ifndef GRADNODE
#define GRADNODE

#include <ros/ros.h>    
#include <ros_coils/Polarity.h>
#include <ros_coils/VI.h>
#include <std_msgs/Float32.h>

class GradPublisher{
private:

    

public:
    GradPublisher(ros::NodeHandle *nh);
    void callbackGradient(const std_msgs::Float32 &msg);
    std_msgs::Float32 grad;
};

#endif