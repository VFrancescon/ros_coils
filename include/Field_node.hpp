#ifndef FIELD_NODE_HPP
#define FIELD_NODE_HPP
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ros/exception.h>

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

    float cal_x = 0.542;  //!< Bx calibration factor. Units are mT/A
    float cal_y = 1.07;  //!< By calibration factor. Units are mT/A
    float cal_z = 0.633;  //!< Bz calibration factor. Units are mT/A
    float ix = 0, iy = 0, iz = 0;
    float bx = 0, by = 0, bz = 0;
    int maxField = 20;
    int maxChange =
        15;  //!< Maximum change in current allowed per cycle. Units are A
    std::vector<std::string> allAddress;
    std::vector<ros_coils::VI> vi_;
};

namespace ros_coils {
    class ChangeException : public ros::Exception {
       public:
        ChangeException(const std::string &message) : ros::Exception(message) {}
    };
    class maxFieldException : public ros::Exception {
       public:
        maxFieldException(const std::string &message)
            : ros::Exception(message) {}
    };
}

#endif  // FIELD_NODE_HPP
