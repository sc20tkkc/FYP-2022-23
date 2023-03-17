#!/usr/bin/env python3
# File used for getting used to controlling and utilising sensors and actuators
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import radians

# Line sensor class (should I seperate actuator and sensor class?)
class LineSensor:
    def __init__(self):
        self.recovering = False
        
        rospy.init_node('line_sensor', anonymous=True)
        self.sub_left = rospy.Subscriber("/robot1/camera_left/rgb/image_raw", Image, callback=self.sensor_callback, queue_size=1)
        self.sub_mid = rospy.Subscriber("/robot1/camera/rgb/image_raw", Image, callback=self.sensor_callback, queue_size=1)
        self.sub_right = rospy.Subscriber("/robot1/camera_right/rgb/image_raw", Image, callback=self.sensor_callback, queue_size=1)
        self.pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)
    
    def sensor_callback(self, image: Image):
        rospy.loginfo(image.data[1])
        cmd = Twist()
        if image.data[1] > 50:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            rospy.sleep(1)
            cmd.linear.x = 0.0
            cmd.angular.z = radians(180)
            self.pub.publish(cmd)
            rospy.sleep(1)
        else:
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
    

if __name__ == "__main__":  
    try:    
        line_sensors = LineSensor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
