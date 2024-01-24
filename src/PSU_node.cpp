#include "psu/PSU_node.hpp"

PSU_node::PSU_node(ros::NodeHandle *nh) {
    std::string COM_PORT;
    if (!ros::param::get("~PSU_COM", COM_PORT)) {
        COM_PORT = "/dev/ttyUSB0";
    }

    if (!ros::param::get("~vConv", vConv)) {
        vConv = 0.01f;
    }
    if (!ros::param::get("~iConv", iConv)) {
        iConv = 0.01f;
    }
    if (!ros::param::get("~debugMode", debugMode)) {
        debugMode = true;
    }

    if (!ros::param::get("~RatedV", RatedV)) {
        RatedV = 50;
    }
    if (!ros::param::get("~RatedI", RatedI)) {
        RatedI = 30;
    }
    vLimit = RatedV * 0.7;
    iLimit = RatedI * 0.7;

    currentI = 0.f;
    currentV = 0.f;
    currentPolarity = 0x01;
    nodeName = ros::this_node::getName();
    ROS_INFO("Debug mode: %s", debugMode ? "true" : "false");
    if (!debugMode) {
        PSU = std::make_unique<DXKDP_PSU>(COM_PORT, vConv, iConv);
    }
    /**
     * @todo add service call to constructor
     *
     */

    powerOnServer_ = nh->advertiseService("powerON" + nodeName,
                                          &PSU_node::callbackSetup, this);

    shutdownServer_ = nh->advertiseService("powerOFF" + nodeName,
                                           &PSU_node::callbackShutdown, this);

    VI_sub_ = nh->subscribe("vi_control" + nodeName, 10,
                            &PSU_node::callbackVIWrite, this);

    if (!debugMode) {
        PSU->PoCtrl(0x01);
    }
    ROS_INFO("Started PSU with port: %s, vConv %f and iConv %f",
             COM_PORT.c_str(), vConv, iConv);
}

void PSU_node::callbackVIWrite(const ros_coils::VI &msg) {
    try {
        if (msg.V > vLimit)
            throw psuExceptions::OverVoltage("Over voltage limit");
        if (msg.I > iLimit)
            throw psuExceptions::OverCurrent("Over current limit");
    } catch (psuExceptions::OverVoltage) {
        std::string outputBuff = this->nodeName +
                                 ": requested V: " + std::to_string(msg.V) +
                                 "V exceeds limit of " + std::to_string(vLimit) +
                                 " Over voltage limit";
        throw(ros::Exception(outputBuff));
    } catch (psuExceptions::OverCurrent) {
        std::string outputBuff = this->nodeName +
                                 ": requested I: " + std::to_string(msg.I) +
                                 "A exceeds limit of " + std::to_string(iLimit) +
                                 " Over current limit";
        throw(ros::Exception(outputBuff));
    }

    int adjCV =
        (int)this->currentV * 100;  // int casted value current held for V
    int adjCI =
        (int)this->currentI * 100;  // int casted value current held for I
    int adjMV = (int)msg.V * 100;   // int casted value message held for V
    int adjMI = (int)msg.I * 100;   // int casted value message held for I
    // print out currentV and msg V
    // ROS_INFO("Current V: %d, msg V: %d", adjCV, adjMV);
    // ROS_INFO("Current I: %d, msg I: %d", adjCI, adjMI);
    bool Vchange = adjCV == adjMV;
    bool Ichange = adjCI == adjMI;

    bool stop_command = Vchange && Ichange;

    switch (debugMode) {
        case false:  // this branch executes if debugMode is false
            if (!stop_command) {
                ROS_INFO("%s: V: %f, I: %f", this->nodeName.c_str(), msg.V,
                         msg.I);
                if (this->nodeName ==
                    "/PSU3") {  // funny gen2 call with polarity embedded
                    // ROS_INFO("Gen2 call");
                    PSU->WriteVIGen2(msg.V, msg.I);
                } else {  // gen1 call with explicit polarity call
                    // ROS_INFO("Gen1 call");
                    PSU->WriteVI(msg.V, abs(msg.I));
                    if (this->nodeName == "/PSU5")
                        PSU->setPolarity(msg.I > 0 ? 0x00 : 0x01);
                    else
                        PSU->setPolarity(msg.I > 0 ? 0x01 : 0x00);
                }
                this->currentI = msg.I;
                this->currentV = msg.V;
            } else {  // no need to act
                // ros_info no need to act
                // ROS_INFO("Input is the same as current Val");
                return;
            }
            break;

        default:  // this branch executes if debugMode is true
            if (!stop_command) {
                ROS_INFO("%s: V: %f, I: %f", this->nodeName.c_str(), msg.V,
                         msg.I);
                if (this->nodeName ==
                    "/PSU3") {  // funny gen2 call with polarity embedded
                    ROS_INFO("Gen2 call");
                } else {  // gen1 call with explicit polarity call
                    ROS_INFO("Gen1 call");
                }
                this->currentI = msg.I;
                this->currentV = msg.V;
            } else {
                // ROS_INFO("Input is the same as current Val");
            }

            break;
    }
}

bool PSU_node::callbackSetup(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res) {
    if (!debugMode) {
        PSU->PoCtrl(0x01);
        PSU->WriteVI(0, 0);
    }
    res.message = "Successfully setup.";
    res.success = true;
    return true;
}
bool PSU_node::callbackShutdown(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res) {
    if (!debugMode) {
        PSU->PoCtrl(0x00);
        PSU->WriteVI(0, 0);
    }
    res.message = "Successfully shutdown.";
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "PSUTEST");
    ros::NodeHandle nh;
    // // ros::AsyncSpinner spinner(2);
    // spinner.start();
    PSU_node psu(&nh);

    while (ros::ok()) {
        ros::spinOnce();
    }
}