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
    ros::Rate loop_rate(1);
    int counter = 0;
    int signFlag = -1;
    ros_coils::magField field;


    while ( ros::ok() )
    {
        if( counter == 20){
            counter = 0; 
            signFlag *= -1;
        }
        field.bx = counter;
        field.by = 0;
        field.bz = 0;
        fieldPublisher.publish(field);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}