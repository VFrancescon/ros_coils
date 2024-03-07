#!/usr/bin/env python
import rospy
from ros_coils.msg import magField

if __name__ == '__main__':
    rospy.init_node('ali_sweep', anonymous=False)

    base_fieldPub = rospy.Publisher('/field', magField, queue_size=1)
    rate = rospy.Rate(0.25) # 0.25hz
    base_field = magField()
    base_field.bx = -5
    base_field.by = 0
    base_field.bz = 0
    sign = 0.5
    rospy.sleep(2.0)
    
    base_fieldPub.publish(base_field)
    rate.sleep()
    while not rospy.is_shutdown():
        base_field.bx = base_field.bx + sign
        if base_field.bx >= 5:
            sign = -0.5
        if base_field.bx <= -5:
            sign = 0.5
        base_fieldPub.publish(base_field)
        rate.sleep()
