#include "pubField.hpp"

FieldPublisher::FieldPublisher(ros::NodeHandle *nh)
{
    // ROSPARAMS here

    // Variable initialisation here

    // Subscriber instantiation here
    fieldSubscriber_ = nh->subscribe("field", 10, &FieldPublisher::callbackField, this);

    // Publisher instantiation here
}

void FieldPublisher::callbackField(const ros_coils::magField &msg)
{
    ROS_INFO("Field: %f, %f, %f", msg.bx, msg.by, msg.bz);
    this->bx = msg.bx;
    this->by = msg.by;
    this->bz = msg.bz;
}

int main(int argc, char *argv[])
{
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
    FieldPublisher field(&nh);
    // trigger the PSU power on service
    ros::ServiceClient poweronClient = nh.serviceClient<std_srvs::Trigger>("powerON/X1");
    std_srvs::Trigger poweronTrigger;
    poweronClient.call(poweronTrigger);

    ros::Publisher x1VIpub = nh.advertise<ros_coils::VI>("/vi_control/X1", 10);
    ros::Publisher x2VIpub = nh.advertise<ros_coils::VI>("/vi_control/X2", 10);
    ros::Publisher y1VIpub = nh.advertise<ros_coils::VI>("/vi_control/Y1", 10);
    ros::Publisher y2VIpub = nh.advertise<ros_coils::VI>("/vi_control/Y2", 10);
    ros::Publisher z1VIpub = nh.advertise<ros_coils::VI>("/vi_control/Z1", 10);
    ros::Publisher z2VIpub = nh.advertise<ros_coils::VI>("/vi_control/Z2", 10);

    ros::Publisher x1Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/X1", 10);
    ros::Publisher x2Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/X2", 10);
    ros::Publisher y1Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Y1", 10);
    ros::Publisher y2Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Y2", 10);
    ros::Publisher z1Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Z1", 10);
    ros::Publisher z2Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Z2", 10);

    ros_coils::Polarity px, py, pz;
    px.Polarity = true;
    py.Polarity = true;
    pz.Polarity = true;

    bool xFlag = true, yFlag = true, zFlag = true;
    ros::Rate freq(1);
    while (ros::ok())
    {
        ros_coils::VI viX, viY, viZ;
        viX.I = field.bx / field.cal_x;
        viY.I = field.by / field.cal_y;
        viZ.I = field.bz / field.cal_z;

        if (viX.I < 0)
        {
            px.Polarity = false;
        }
        if (viY.I < 0)
        {
            py.Polarity = false;
        }
        if (viX.I < 0)
        {
            pz.Polarity = true;
        }

        if (px.Polarity != xFlag)
        {
            x1Polaritypub.publish(px);
            x2Polaritypub.publish(px);
            xFlag = px.Polarity;
        }

        if (py.Polarity != yFlag)
        {
            y1Polaritypub.publish(py);
            y2Polaritypub.publish(py);
            yFlag = py.Polarity;
        }

        if (pz.Polarity != zFlag)
        {
            z1Polaritypub.publish(pz);
            z2Polaritypub.publish(pz);
            zFlag = pz.Polarity;
        }

        viX.V = viX.I * 2;
        viY.V = viY.I * 2;
        viZ.V = viZ.I * 2;

        x1VIpub.publish(viX);
        x2VIpub.publish(viX);
        y1VIpub.publish(viY);
        y2VIpub.publish(viY);
        z1VIpub.publish(viZ);
        z2VIpub.publish(viZ);

        // x1VIpub.publish()
        ros::spinOnce();
        freq.sleep();
    }

    // trigger the poweroff trigger
    ros::ServiceClient shutdownClient = nh.serviceClient<std_srvs::Trigger>("powerOFF/X1");
    std_srvs::Trigger shutdownTrigger;
    shutdownClient.call(shutdownTrigger);

    return 0;
}