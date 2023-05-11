#include "grad_node.hpp"

GradPublisher::GradPublisher(ros::NodeHandle *nh)
{
    this->grad.data = 0.f;
    // Variable initialisation here

    nh->subscribe("gradient", 10, &GradPublisher::callbackGradient, this);
    // Subscriber instantiation here


    // Publisher instantiation here
}

void GradPublisher::callbackGradient(const std_msgs::Float32 &msg)
{
    ROS_INFO("Gradient: %f", msg.data);
    this->grad.data = msg.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "GradPub");
    ros::NodeHandle nh;

    

    GradPublisher grad(&nh);

    ros::Rate freq(0.25);

    while(ros::ok()){

        


        ros::spinOnce();
        freq.sleep();
    }


    return 0;
}