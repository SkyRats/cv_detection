#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

def run():

    rospy.init_node("head")
    rate = rospy.Rate(30)
    cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)

    """ INITIALIZE CV_CONTROL """
    for i in range (10):
        cv_control_publisher.publish(Bool(True))
        rate.sleep()


    init_time = rospy.get_rostime().secs
    while rospy.get_rostime().secs - init_time <= 60:
        rate.sleep()

    """ END CV CONTROL """
    for i in range(10):
        rospy.logwarn("Deactivating CV Control")
        cv_control_publisher.publish(Bool(False))
        rate.sleep()

if __name__ == "__main__":
    run()
    