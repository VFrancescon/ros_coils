#!/usr/bin/env python
import rospy
from ros_coils.msg import magField
import numpy as np
import csv
import rospkg

if __name__ == '__main__':
    rospy.init_node('ali_sweep', anonymous=False)

    path_to_pkg = rospkg.RosPack().get_path('ros_coils')
    print(path_to_pkg)

    csv_rate = rospy.get_param('csv_rate', 0.125)
    csv_path = rospy.get_param('csv_path', 'csv/sample_csv.csv')
    rate = rospy.Rate(csv_rate)  # 0.125 Hz

    bx = []
    by = []
    bz = []
    csv_path = path_to_pkg + '/csv/' + csv_path
    with open(csv_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            try:
                bx.append(float(row[0]))
                by.append(float(row[1]))
                bz.append(float(row[2]))
            except ValueError as e:
                line_number = reader.line_num
                raise ValueError(f"Invalid value in CSV file. All values must be numbers. Error occurred in line {line_number}.") from e

    fieldPub = rospy.Publisher('/field', magField, queue_size=1)
    field = magField()
    
    for i in range(len(bx)):
        field.bx = bx[i]
        field.by = by[i]
        field.bz = bz[i]
        field.header.stamp = rospy.Time.now()
        fieldPub.publish(field)
        rate.sleep()
        print(field.bx, field.by, field.bz)