#include <ros/ros.h>
#include <ros_coils/Polarity.h>
#include <ros_coils/VI.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "yGrads");

    ros::NodeHandle nh;

    ros::Publisher y1VIpub = nh.advertise<ros_coils::VI>("/vi_control/Y1", 10);
    ros::Publisher y2VIpub = nh.advertise<ros_coils::VI>("/vi_control/Y2", 10);
    ros::Publisher y1Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Y1", 10);
    ros::Publisher y2Polaritypub = nh.advertise<ros_coils::Polarity>("/polarity_control/Y2", 10);

    ros_coils::Polarity py1, py2;
    ros_coils::VI viY1, viY2;

    py1.Polarity = 0x01;
    py2.Polarity = 0x00;
    ros::Rate loop_rate(3.5);

    viY1.V = 40.f;
    viY2.V = 40.f;
    viY1.I = 0.f;
    viY2.I = 0.f;
    int counter = 0;

    while(ros::ok()){

        switch(counter){
            case 0:
                viY1.V = 40.f;
                viY2.V = 40.f;
                viY1.I = 25.f;
                viY2.I = 25.f;
                py1.Polarity = 0x01;
                py2.Polarity = 0x01;
            break;

            case 1:
                viY1.V = 40.f;
                viY2.V = 40.f;
                viY1.I = 25.f;
                viY2.I = 25.f;
                py1.Polarity = 0x00;
                py2.Polarity = 0x00;
            break;


            // case 2:
            //     viY1.V = 40.f;
            //     viY2.V = 40.f;
            //     viY1.I = 25.f;
            //     viY2.I = 25.f;
            //     py1.Polarity = 0x00;
            //     py2.Polarity = 0x01;
            // break;

            default:
                viY1.V = 40.f;
                viY2.V = 40.f;
                viY1.I = 25.f;
                viY2.I = 25.f;
                py1.Polarity = 0x01;
                py2.Polarity = 0x01;
                counter = 0;

            break;
        }


        y1VIpub.publish(viY1);
        y2VIpub.publish(viY2);
        y1Polaritypub.publish(py1);
        y2Polaritypub.publish(py2);
        
        //usleep one second

        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}