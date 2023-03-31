#include "pubField.hpp"

FieldPublisher::FieldPublisher(ros::NodeHandle *nh)
{
    // ROSPARAMS here

    // Variable initialisation here

    // Subscriber instantiation here
    fieldSubscriber_ = nh->subscribe("field", 10, &FieldPublisher::callbackField, this);
    this->bx = 0;
    this->by = 0;
    this->bz = 0;
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
    // ros::ServiceClient poweronClient = nh.serviceClient<std_srvs::Trigger>("powerON/X1");
    // std_srvs::Trigger poweronTrigger;
    // poweronClient.call(poweronTrigger);

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
    // ros::Publisher z2Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Z2", 10);

    ros_coils::Polarity px, py, pz;
    px.Polarity = 0x01;
    py.Polarity = 0x01;
    pz.Polarity = 0x01;

    ros::AsyncSpinner spinner(6);
    ros::Rate freq(0.5);
    ros_coils::VI viX, viY, viZ1, viZ2;

    viX.I = 0.f;
    viX.V = 0.f;

    viY.I = 0.f;
    viY.V = 0.f;

    viZ1.I = 0.f;
    viZ1.V = 0.f;

    viZ2.I = 0.f;
    viZ2.V = 0.f;

    while (ros::ok())
    {
        viX.I = field.bx / field.cal_x;
        viY.I = field.by / field.cal_y;
        viZ1.I = field.bz / field.cal_z;
        viZ2.I = field.bz / field.cal_z;

        if (viX.I < 0)
        {
            px.Polarity = 0x00;
        }
        else
            px.Polarity = 0x01;
        if (viY.I < 0)
        {
            py.Polarity = 0x00;
        }
        else
            py.Polarity = 0x01;
        if (viZ1.I < 0)
        {
            pz.Polarity = 0x00;
        }
        else
            pz.Polarity = 0x01;

        // if (px.Polarity != xFlag)
        // {
            x1Polaritypub.publish(px);
            x2Polaritypub.publish(px);
             
        // }

        // if (py.Polarity != yFlag)
        // {
            y1Polaritypub.publish(py);
            y2Polaritypub.publish(py);
        // }

        // if (pz.Polarity != zFlag)
        // {
            z1Polaritypub.publish(pz);
            // z2Polaritypub.publish(pz);
        // }
        viX.I = abs(viX.I);
        viY.I = abs(viY.I);
        viZ1.I = abs(viZ1.I);
        //z2 is unchanged

        viX.V = abs(viX.I);
        viY.V = abs(viY.I);
        viZ1.V = abs(viZ1.I);
        viZ2.V = abs(viZ2.I);

        // std::cout << "viX: " << viX.V << "," << viX.I << "\n";
        // std::cout << "viY: " << viY.V << "," << viY.I << "\n";
        // std::cout << "viZ: " << viZ.V << "," << viZ.I << "\n";
        // printf("px= %02X\n", px.Polarity);
        // printf("py= %02X\n", py.Polarity);
        // printf("pz= %02X\n", pz.Polarity);


        x1VIpub.publish(viX);
        x2VIpub.publish(viX);
        y1VIpub.publish(viY);
        y2VIpub.publish(viY);
        z1VIpub.publish(viZ1);
        z2VIpub.publish(viZ2);

        spinner.start();
        freq.sleep();
    }

    // std::cout << "Yup. end of program is triggered\n";
    // trigger the poweroff trigger
    // ros_coils::VI exitVI;
    // exitVI.V = 0;
    // exitVI.I = 0;
    // x1VIpub.publish(exitVI);
    // x2VIpub.publish(exitVI);
    // ros::spinOnce();
    // y1VIpub.publish(exitVI);
    // y2VIpub.publish(exitVI);
    // z1VIpub.publish(exitVI);
    // z2VIpub.publish(exitVI);

    ros::ServiceClient shutdownClient = nh.serviceClient<std_srvs::Trigger>("powerOFF/X1");
    std_srvs::Trigger shutdownTrigger;
    shutdownClient.call(shutdownTrigger);

    return 0;
}