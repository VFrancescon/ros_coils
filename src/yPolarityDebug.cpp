#ifndef Y_POLARITY_DEBUG_H
#define Y_POLARITY_DEBUG_H

#include <ros/ros.h>
#include <memory>
#include <sstream>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

#include "ros_coils/VI.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "yPolarityDebug");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);


    int yNum = 2;
    std::string yRoot;
    std::vector<std::string> yAddress;
    if (!ros::param::get("~yRoot", yRoot)) {  // first address of PSU for Y axis
        yRoot = "/PSU4";
    }
    for (int i = 0; i < yNum; i++) {
        std::string rootDuplicate = yRoot;
        char num = yRoot[4];
        num += i;
        rootDuplicate[4] = num;
        rootDuplicate = "/vi_control" + rootDuplicate;
        yAddress.push_back(rootDuplicate);
    }
        
    std::vector<ros::Publisher> viPub_;

    for(auto i: yAddress){
        viPub_.push_back(nh.advertise<ros_coils::VI>(i, 10));
    }

    ros_coils::VI msg1, msg2;
    msg1.I = 10;
    msg1.V = 10;
    msg2.I = 10;
    msg2.V = 10;

    int test_case = 0;
    int test_case_max = 3;

    while(ros::ok()){
        // Take user input inside the while loop
        std::cout << "Enter 1 to advance cases: ";
        std::string input;
        std::cin >> input;


        // If nothing happens, proceed with the loop
        if (!input.empty()) {
            if(stoi(input) == 1) test_case++;
            if (test_case > test_case_max) {
                test_case = 0;
            }
        }
        switch (test_case)
        {
        case 1: // PP
            msg1.I = 10;
            msg2.I = 10;
            continue;
            break;
        case 2: // NP
            msg1.I = -10;
            msg2.I = 10;    
            break;
        case 3: // NN
            msg1.I = -10;
            msg2.I = -10;
            break;
        default: // PN
            msg1.I = 10;
            msg2.I = -10;
            break;
        }
        ROS_INFO("Case %d, I1 = %f, I2 = %f", test_case, msg1.I, msg2.I);
        // Publish messages
        viPub_[0].publish(msg1);
        viPub_[1].publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


#endif // Y_POLARITY_DEBUG_H