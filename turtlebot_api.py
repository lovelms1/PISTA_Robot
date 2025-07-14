#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class TurtleBotAPI:
    def __init__(self):
        rospy.init_node('turtlebot_api_node', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.lidar_data = None

    def move_forward(self, speed=0.1, duration=2):
        twist_msg = Twist()
        twist_msg.linear.x = speed
        rate = rospy.Rate(10)
        end_time = rospy.Time.now() + rospy.Duration(duration)

        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()
        self.stop()

    def rotate(self, angle_in_degrees=90, angular_speed_deg=30):
        twist_msg = Twist()
        angular_speed_rad = math.radians(abs(angular_speed_deg))
        angle_rad = math.radians(abs(angle_in_degrees))

        twist_msg.angular.z = angular_speed_rad if angle_in_degrees > 0 else -angular_speed_rad
        duration = angle_rad / angular_speed_rad
        rate = rospy.Rate(10)
        end_time = rospy.Time.now() + rospy.Duration(duration)

        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()
        self.stop()

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    def lidar_callback(self, data):
        self.lidar_data = data.ranges

    def get_lidar_scan(self):
        return self.lidar_data

if __name__ == '__main__':
    bot = TurtleBotAPI()
    rospy.sleep(1)

    print("Moving forward...")
    bot.move_forward(speed=0.2, duration=3)

    print("Rotating...")
    bot.rotate(angle_in_degrees=90)

    scan = bot.get_lidar_scan()
    if scan:
        print("LiDAR scan sample distances:", scan[:10])
    else:
        print("No LiDAR data received.")
