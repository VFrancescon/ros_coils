#ifndef FIELD_NODE_HPP
#define FIELD_NODE_HPP
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <sstream>
#include <vector>

#include "ros_coils/VI.h"
#include "ros_coils/magField.h"

class FieldNode {
   private:
    ros::Subscriber field_Subscriber_;
    ros::Timer field_subscriber_timer_;
    double field_subscriber_frequency_;

    /**
     * @brief Number of supplies granted to a given axis.
     *
     */
    int xNum = 2, yNum = 2, zNum = 2;

    /**
     * @brief A note on the _Root variables:
     * If _Num is set to 1, _Root is simply the address of the supply powering
     * the axis. Else, _Root is the address of the first supply powering the
     * axis, with the second being the immediate successor. EG: xNum = 2,
     * xRoot="/PSU0", X1 coil will be powered by /PSU0, X2 coil will be powered
     * by /PSU1
     *
     */
    std::string xRoot, yRoot, zRoot;

    std::vector<std::string> xAddress, yAddress, zAddress;

    void field_to_vi();

   public:
    FieldNode(ros::NodeHandle *nh);
    void callbackField(const ros_coils::magField &msg);

    float cal_x = 0.53;  //!< Bx calibration factor. Units are mT/A
    float cal_y = 1.07;  //!< By calibration factor. Units are mT/A
    float cal_z = 0.62;  //!< Bz calibration factor. Units are mT/A
    float ix, iy, iz;
    float bx, by, bz;
    std::vector<std::string> allAddress;
    std::vector<ros_coils::VI> vi_;

};


#endif  // FIELD_NODE_HPP
