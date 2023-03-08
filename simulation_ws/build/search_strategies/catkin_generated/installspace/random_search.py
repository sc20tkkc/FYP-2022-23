import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import radians

def sensor_callback(data):
    rate = rospy.Rate(10)
    rospy.loginfo(data.data[1])
    cmd = Twist()
    if(data.data[1] > 70):
        cmd.linear.x = 0.0
        cmd.angular.z = radians(90)
    else:
        cmd.linear.x = -0.5

    pub.publish(cmd)


if __name__ == "__main__":
    try:
        
        rospy.init_node("random_search")
        pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
        sub_mid = rospy.Subscriber("/robot1/camera/rgb/image_raw", Image, callback=sensor_callback)
        rospy.loginfo("Node has been started")
        while not rospy.is_shutdown():
            rospy.spin()    
        
    except rospy.ROSInterruptException:
        pass
