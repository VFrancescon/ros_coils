#include "psu/psuTest.hpp"

PsuROSWrapper::PsuROSWrapper(ros::NodeHandle *nh)
{
    std::string COM_PORT;
    if (!ros::param::get("~PSU_COM", COM_PORT))
    {
        COM_PORT = "/dev/ttyUSB0";
    }

    if (!ros::param::get("~vConv", vConv))
    {
        vConv = 0.01f;
    }
    if (!ros::param::get("~iConv", iConv))
    {
        iConv = 0.01f;
    }

    currentI = 0;
    currentV = 0;
    currentPolarity = true;
    nodeName = ros::this_node::getName();

    // X1_ = std::make_unique<DXKDP_PSU>(COM_PORT, vConv, iConv);
    ROS_INFO("Started PSU with port: %s, vConv %f and iConv %f",
             COM_PORT.c_str(), vConv, iConv);

    // currentSubscriber_ = nh->subscribe(
    //     "current_control" + nodeName, 10, &PsuROSWrapper::callbackCurrentWrite, this);

    // voltageSubscriber_ = nh->subscribe(
    //     "voltage_control" + nodeName, 10, &PsuROSWrapper::callbackVoltageWrite, this);

    viSubscriber_ = nh->subscribe(
        "vi_control" + nodeName, 10, &PsuROSWrapper::callbackVIWrite, this);

    polaritySubscriber_ = nh->subscribe(
        "polarity_control" + nodeName, 10, &PsuROSWrapper::callbackPolarity, this
    );
    
    powerOnServer_ = nh->advertiseService(
        "powerON" + nodeName, &PsuROSWrapper::callbackSetup, this);

    shutdownServer_ = nh->advertiseService(
        "powerOFF" + nodeName, &PsuROSWrapper::callbackShutdown, this);
}

void PsuROSWrapper::callbackCurrentWrite(const std_msgs::Float32 &msg)
{
    bool enactCommand = compare_float(this->currentI, msg.data, this->iConv);

    if (enactCommand)
    {
        ROS_INFO("Input is the same as current Val.No need to change current.");
    }
    else
    {
        ROS_INFO("Setting Current to: %f", msg.data);
        // X1_->WriteCurrent(msg.data);
        this->currentI = msg.data;
    }
}

void PsuROSWrapper::callbackVoltageWrite(const std_msgs::Float32 &msg)
{
    bool enactCommand = compare_float(this->currentV, msg.data, this->vConv);

    if (enactCommand)
    {
        ROS_INFO("Input is the same as current Val.No need to change voltage.");
        return;
    }
    else
    {
        ROS_INFO("Setting Voltage to: %f", msg.data);
        // X1_->WriteVoltage(msg.data);
        this->currentV = msg.data;
    }
}

void PsuROSWrapper::callbackVIWrite(const ros_coils::VI &msg)
{
    bool Vchange = msg.V == this->currentV;
    bool Ichange = msg.I == this->currentI;

    if (Vchange && Ichange)
    {
        ROS_INFO("PSU: %s. No need to do anything. Values are unchanged", this->nodeName.c_str());
    }
    else
    {
        ROS_INFO("PSU: %s. Setting V=%f, I=%f", this->nodeName.c_str(), msg.V, msg.I);
        // X1_->WriteVI(msg.V, msg.I);
        this->currentV = msg.V;
        this->currentI = msg.I;
    }
}

void PsuROSWrapper::callbackPolarity(const ros_coils::Polarity &msg)
{
    if(msg.Polarity == this->currentPolarity){
        ROS_INFO("PSU: %s. No need to act", this->nodeName.c_str());
        return;
    } else {
        ROS_INFO("PSU: %s. Setting polarity to %d",this->nodeName.c_str(), msg.Polarity);
        
        if( this->nodeName == "/Z2" ){
            ROS_INFO("Using GEN2");
            // X1_->setPolarityGen2(msg.Polarity);
        } else {
            // X1_->setPolarity(msg.Polarity, 0x01);
        }
        this->currentPolarity = msg.Polarity;
    }
}

bool PsuROSWrapper::callbackSetup(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // X1_->PoCtrl(0x01);
    res.message = "Successfully setup.";
    res.success = true;
    return true;
}

bool PsuROSWrapper::callbackShutdown(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // X1_->PoCtrl(0x00);
    res.message = "Successfully setup.";
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PSUTEST");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(6);
    spinner.start();
    PsuROSWrapper psu1(&nh);

    ros::waitForShutdown();
}