#!/usr/bin/env python
import rospy
from ros_coils.msg import magField

if __name__ == '__main__':
    maxAbs = 15
    rospy.init_node('ali_sweep', anonymous=False)

    base_fieldPub = rospy.Publisher('/field', magField, queue_size=1)
    rate = rospy.Rate(1/15)  # 0.125 Hz
    base_field = magField()
    base_field.bx = -5
    base_field.by = 0
    base_field.bz = 0
    sign = 2.5
    base_fieldPub.publish(base_field)
    rospy.sleep(2.0)

    base_field.bx = -1 * maxAbs
    base_field.by = 0
    base_field.bz = 0
    base_fieldPub.publish(base_field)
    rate.sleep()
    while not rospy.is_shutdown():
        base_field.bx = base_field.bx + sign
        if base_field.bx >= maxAbs:
            sign = -2.5
        if base_field.bx <= maxAbs * -1:
            sign = 2.5
        base_fieldPub.publish(base_field)
        rate.sleep()
