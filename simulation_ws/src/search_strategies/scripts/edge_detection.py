#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import radians

def sensor_callback(image: Image):
    rate = rospy.Rate(30)
    turning = False
    rospy.loginfo(image.data[1])
    cmd = Twist()
    if(image.data[1] > 70):
        cmd.linear.x = 0.0
        cmd.angular.z = radians(180)
        turning = True
    else:
        cmd.linear.x = -0.2
        cmd.angular.z = 0.0
        
    pub.publish(cmd)
    
    if turning:
        rate.sleep()


if __name__ == "__main__":  
    try:
        rospy.init_node("edge_detection")
        pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)
        sub_left = rospy.Subscriber("/robot1/camera_left/rgb/image_raw", Image, callback=sensor_callback)
        sub_mid = rospy.Subscriber("/robot1/camera/rgb/image_raw", Image, callback=sensor_callback)
        sub_right = rospy.Subscriber("/robot1/camera_right/rgb/image_raw", Image, callback=sensor_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pub
        pass
