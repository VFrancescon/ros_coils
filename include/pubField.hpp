#ifndef PUBFIELD
#define PUBFIELD
#include <memory>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "ros_coils/magField.h"
#include "ros_coils/Polarity.h"
#include "ros_coils/VI.h"

class FieldPublisher{
private:

        
    //Publishers
    ros::Publisher xVIpub_;
    
    //Subscribers
    ros::Subscriber fieldSubscriber_;

    //Timers & Frequencies 
    ros::Timer field_subscriber_timer_;
    double field_subscriber_frequency_;



public:
    FieldPublisher(ros::NodeHandle *nh);
    void callbackField(const ros_coils::magField &msg);

    float cal_x = 0.53; //!< Bx calibration factor. Units are mT/A
    float cal_y = 1.07; //!< By calibration factor. Units are mT/A
    float cal_z = 0.62; //!< Bz calibration factor. Units are mT/A
    float ix, iy, iz;
    float bx, by, bz;
};

int main(int argc, char* argv[]);

#endif