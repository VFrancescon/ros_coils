#include "psu/psuTest.hpp"

PsuROSWrapper::PsuROSWrapper(ros::NodeHandle *nh)
{
    std::string COM_PORT;
    if (!ros::param::get("~PSU_COM", COM_PORT))
    {
        COM_PORT = "/dev/ttyUSB0";
    }
    float vConv, iConv;
    if (!ros::param::get("~vConv", vConv))
    {
        vConv = 0.1;
    }
    if (!ros::param::get("~iConv", iConv))
    {
        iConv = 0.01;
    }

    currentI = 0;
    currentV = 0;

    // X1_ = std::make_unique<DXKDP_PSU>(COM_PORT, vConv, iConv);
    ROS_INFO("Started PSU with port: %s, vConv %f and iConv %f",
            COM_PORT.c_str(), vConv, iConv);

    currentSubscriber_ = nh->subscribe(
        "current_control", 10, &PsuROSWrapper::callbackCurrentWrite, this);

    voltageSubscriber_ = nh->subscribe(
        "voltage_control", 10, &PsuROSWrapper::callbackVoltageWrite, this);
}

void PsuROSWrapper::callbackCurrentWrite(const std_msgs::Int32 &msg)
{   
    int inputData = msg.data;

    ROS_INFO("Input is: %d. Same condition flag: %d", msg.data, currentI == inputData);
    if ( currentI == inputData )
    {
        ROS_INFO("Input is the same as current Val.No need to change current.");
    }
    else
    {
        ROS_INFO("Setting Current to: %d", inputData);
        // X1_->WriteCurrent(inputData);
    }
}

void PsuROSWrapper::callbackVoltageWrite(const std_msgs::Int32 &msg)
{   
    int inputData = msg.data;
    ROS_INFO("Input is: %d. Same condition flag: %d", msg.data, currentV == inputData);
    if ( currentV == inputData )
    {
        ROS_INFO("Input is %d the same as current Val %d .No need to change Voltage.",
            currentV, inputData);
    }
    else
    {
        ROS_INFO("Setting Voltage to: %d", inputData);
        // X1_->WriteVoltage(inputData);
    }
}

void PsuROSWrapper::callbackSetup(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // X1_->PoCtrl(0x01);
}

void PsuROSWrapper::callbackShutdown(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // X1_->PoCtrl(0x00);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PSUTEST");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    PsuROSWrapper psu1(&nh);
    


    ros::waitForShutdown();
}