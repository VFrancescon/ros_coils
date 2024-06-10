#!/usr/bin/env python
import rospy
import time
from ros_coils.msg import magField
from ros_coils.msg import Polarity
from ros_coils.msg import VI

if __name__ == '__main__':
    rospy.init_node('test_transitions', anonymous=False)
    VI_1 = VI()
    VI_1.voltage = 10
    VI_1.current = 5
    VI_2 = VI_1

    vi1_pub = rospy.Publisher('/vi_control/PSU0', VI, queue_size=1)
    vi2_pub = rospy.Publisher('/vi_control/PSU1', VI, queue_size=1)
    rate = rospy.Rate(1/15)  # 0.125 Hz
    vi1_pub.publish(VI_1)
    vi2_pub.publish(VI_2)

    Polarity_1 = Polarity()
    Polarity_1.polarity = 0
    Polarity_2 = Polarity_1

    valid_polarity = [0, 1, 2, 3]

    polarity1_pub = rospy.Publisher('/polarity/PSU0', Polarity, queue_size=1)
    polarity2_pub = rospy.Publisher('/polarity/PSU1', Polarity, queue_size=1)

    for initial_polarity in valid_polarity:
        for final_polarity in valid_polarity:
            print(f"Testing transition from {initial_polarity} to {final_polarity}")
            Polarity_1.polarity = initial_polarity
            polarity1_pub.publish(Polarity_1)
            time.sleep(0.5)  # wait for the transition to be processed
            Polarity_1.polarity = final_polarity
            polarity1_pub.publish(Polarity_1)
            input("Press Enter to continue...")

    while not rospy.is_shutdown():
        pass

