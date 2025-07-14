#!/usr/bin/env python3
import rospy
import time
from turtlebot3_api.api import TurtleBotAPI

def drive_square(bot, side_length=1.0, speed=0.1):
    for _ in range(4):
        # Move straight
        print("Moving straight...")
        bot.move(linear_speed=speed, duration=side_length/speed)
        time.sleep(1)  # pause briefly

        # Rotate 90 degrees
        print("Rotating 90 degrees...")
        bot.move(angular_speed=0.5, duration=3.14)  # Approximate 90-degree turn
        time.sleep(1)

        # Print closest obstacle distance
        lidar_data = bot.get_lidar()
        if lidar_data:
            print("Closest obstacle:", min(lidar_data))
        else:
            print("No LiDAR data received.")

if __name__ == "__main__":
    rospy.init_node("turtlebot_cli_demo", anonymous=True)
    bot = TurtleBotAPI()
    rospy.sleep(1)
    drive_square(bot)
