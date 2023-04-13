#include <memory>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "ros_coils/magField.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "FieldaPub");
    ros::NodeHandle nh;

    ros::Publisher fieldPublisher = nh.advertise<ros_coils::magField>("/field", 10);
    ros_coils::magField field0, field1, field2, field3;
    field0.bx = 0;
    field0.by = 0;
    field0.bz = 0;

    field1.bx = 10;
    field1.by = 10;
    field1.bz = -10;    
    
    field2.bx = -10;
    field2.by = -5;
    field2.bz = 10;    
    
    field3.bx = 0;
    field3.by = 0;
    field3.bz = -10;    
    
    ros::Rate freq(0.1);
    int counter = 0;
    while(ros::ok()){
        if(counter == 0){
            fieldPublisher.publish(field0);
        } else if( counter == 1){
            fieldPublisher.publish(field1);
        } else if( counter == 2){
            fieldPublisher.publish(field2);
        } else if( counter == 3){
            fieldPublisher.publish(field3);
        } else {
            counter = 0; 
            ros::spinOnce();
            freq.sleep();
            continue;}

        counter++;
        ros::spinOnce();
        freq.sleep();
    }

    return 0;
}