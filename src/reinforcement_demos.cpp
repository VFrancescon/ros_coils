#include <ros/ros.h>
#include <memory>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include "ros_coils/magField.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "ReinforcementShowcase");
    ros::NodeHandle nh;
    ros::Publisher fieldPublisher = nh.advertise<ros_coils::magField>("/field", 10);
    ros::Rate loop_rate(0.5);
    int counter = 0;
    int adder = 1;
    ros_coils::magField field;
    field.by = 0;
    field.bz = 0;
        

    // field = 0
    // var = 1
    // if (field == 20 ) var = -1
    // if ( field == -20 ) var = 1
    // field = field + var; 

    while ( ros::ok() )
    {
        if(counter == 20 ) adder = -1;
        if(counter == -20 ) adder = 1;

        counter = counter + adder;
        field.bx = counter;
        fieldPublisher.publish(field);
        ros::spinOnce();
        loop_rate.sleep();;
    }
    

    return 0;
}

/**
 * @todo: rewrite this to work with the new field message 
 * 
 */