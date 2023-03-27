#include "pubField.hpp"

FieldPublisher::FieldPublisher(ros::NodeHandle *nh){
    //ROSPARAMS here

    //Variable initialisation here

    //Subscriber instantiation here
    fieldSubscriber_ = nh->subscribe("field", 10, &FieldPublisher::callbackField, this);

    //Publisher instantiation here

}

void FieldPublisher::callbackField(const ros_coils::magField &msg){
    ROS_INFO("Field: %f, %f, %f", msg.bx, msg.by, msg.bz);
    this->bx = msg.bx;
    this->by = msg.by;
    this->bz = msg.bz;
}

void FieldPublisher::publishField(const ros::TimerEvent &event){

}


int main(int argc, char* argv[]){
    // float bx = 10, by = 10, bz = 10;

    // if(argc == 4){
    //     bx = std::stof(argv[1]);
    //     by = std::stof(argv[2]);
    //     bz = std::stof(argv[3]);
    // }

    // float ix, iy, iz;

    // ix = bx / cal_x;
    // iy = by / cal_y;
    // iz = bz / cal_z;

    ros::init(argc, argv, "FieldPub");
    ros::NodeHandle nh;
    //trigger the PSU power on service
    ros::ServiceClient poweronClient = nh.serviceClient<std_srvs::Trigger>("powerON/X1");
    std_srvs::Trigger poweronTrigger;
    poweronClient.call(poweronTrigger);

    // // ros::Publisher xVIpub_ = nh.advertise<ros_coils::VI>("/vi_control/X1", 10);

    
    // ros::Subscriber Bxsub_;
    // ros::Subscriber Bysub_;
    // ros::Subscriber Bzsub_;
    
    // ros::Rate loop_rate(1);
    // while(ros::ok()){


    // }

    //trigger the poweroff trigger
    ros::ServiceClient shutdownClient = nh.serviceClient<std_srvs::Trigger>("powerOFF/X1");
    std_srvs::Trigger shutdownTrigger;
    shutdownClient.call(shutdownTrigger);

    return 0;
}