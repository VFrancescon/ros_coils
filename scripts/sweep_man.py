#!/usr/bin/env python
import rospy
from ros_coils.msg import magField
import numpy as np
if __name__ == '__main__':

    sweepRate = 0.125
    increment = 2.5
    rospy.init_node('ali_sweep', anonymous=False)

    maxAbs = rospy.get_param('maxAbs', [0,0,0])
    sweepRate = rospy.get_param('sweepRate', 1/15)
    increment = rospy.get_param('increment', 2.5)
    maxAbs = np.array(maxAbs, dtype=float)

    base_fieldPub = rospy.Publisher('/field', magField, queue_size=1)
    rate = rospy.Rate(sweepRate)  # 0.125 Hz
    base_field = magField()
    base_field.bx = -maxAbs[0]/2
    base_field.by = -maxAbs[1]/2
    base_field.bz = -maxAbs[1]/2
    base_field.header.stamp = rospy.Time.now()
    base_fieldPub.publish(base_field)
    rospy.sleep(2.0)

    x_incr = increment if maxAbs[0] != 0 else 0
    y_incr = increment if maxAbs[1] != 0 else 0
    z_incr = increment if maxAbs[2] != 0 else 0

    base_field.bx = -1 * maxAbs[0]
    base_field.by = -1 * maxAbs[1]
    base_field.bz = -1 * maxAbs[2]
    rospy.loginfo("Sweeping from: " + str(base_field.bx) + " " + str(base_field.by) + " " + str(base_field.bz))
    
    base_fieldPub.publish(base_field)
    rate.sleep()
    while not rospy.is_shutdown():
        base_field.bx = base_field.bx + x_incr
        base_field.by = base_field.by + y_incr
        base_field.bz = base_field.bz + z_incr
        
        if base_field.bx >= maxAbs[0] or base_field.bx <= maxAbs[0] * -1:
            x_incr = -x_incr

        if base_field.by >= maxAbs[1] or base_field.by <= maxAbs[1] * -1:
            y_incr = -y_incr

        if base_field.bz >= maxAbs[2] or base_field.bz <= maxAbs[2] * -1:
            z_incr = -z_incr

        # if base_field.bx <= maxAbs[0] * -1:
        #     x_incr = -x_incr
        base_fieldPub.publish(base_field)
        rate.sleep()
