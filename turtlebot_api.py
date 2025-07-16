import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math

class TurtleBotAPI(Node):
    def __init__(self):
        super().__init__('turtlebot3_api_node')
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscriber for LiDAR scans
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.lidar_data = None

    def move(self, linear_speed=0.0, angular_speed=0.0, duration=2.0):
        """Publish linear and angular speeds for a given duration (seconds)."""
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        end_time = self.get_clock().now() + Duration(seconds=duration)
        rate = self.create_rate(10)
        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_vel_pub.publish(msg)
            rate.sleep()
        # Stop the robot after movement
        self.stop()

    def stop(self):
        """Publish zero velocities to stop the robot."""
        self.cmd_vel_pub.publish(Twist())

    def lidar_callback(self, msg: LaserScan):
        """Callback to store latest LiDAR ranges."""
        self.lidar_data = msg.ranges

    def get_lidar(self):
        """Return the latest LiDAR scan data."""
        return self.lidar_data


def drive_square(bot, side_length=1.0, speed=0.1):
    """Drive the robot in a square pattern and print the closest obstacle distance."""
    for _ in range(4):
        print("Moving straight...")
        bot.move(linear_speed=speed, duration=side_length/speed)
        time.sleep(1.0)

        print("Rotating 90 degrees...")
        bot.move(angular_speed=0.5, duration=(math.pi/2) / 0.5)
        time.sleep(1.0)

        scan = bot.get_lidar()
        if scan:
            print("Closest obstacle:", min(scan))
        else:
            print("No LiDAR data received.")


def main():
    """Initialize ROS 2, create the API object, run the square drive demo, and shutdown."""
    rclpy.init()
    bot = TurtleBotAPI()
    # Allow subscriptions to initialize
    rclpy.spin_once(bot, timeout_sec=1.0)

    drive_square(bot)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
