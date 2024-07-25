#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/quori/base_controller/cmd_vel', Twist, queue_size=10)
        # self.cmd_vel_pub = rospy.Publisher('/quori/base/cmd_diff', Twist, queue_size=10)
        rospy.Subscriber('scan_filtered', LaserScan, self.laser_scan_callback)

        self.closest_obstacle_distance = float('inf')
        self.closest_obstacle_angle = 0.0
        self.safe_distance = 0.35  # Safe distance to avoid obstacles
        self.laser_scan_data = None  # Initialize laser_scan_data
        self.start_turn_distance = 0.7  # Start turning when the closest obstacle is at this distance

    def laser_scan_callback(self, data):
        self.laser_scan_data = data  # Update laser_scan_data with the latest data
        self.closest_obstacle_distance = min(data.ranges)
        self.closest_obstacle_angle = data.ranges.index(self.closest_obstacle_distance)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            twist_msg = Twist()

            print('Closest obstacle distance: ', self.closest_obstacle_distance)
            if self.closest_obstacle_distance > self.safe_distance and self.closest_obstacle_distance > self.start_turn_distance:
                print('Move forward')   
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.0
                
            elif self.start_turn_distance > self.closest_obstacle_distance > self.safe_distance:
                print('Start turning')
                twist_msg.linear.x = 0.2  # Move forward
                twist_msg.angular.z = 0.0  # No rotation
                # Use self.closest_obstacle_angle to determine the direction to turn
                if self.laser_scan_data and self.closest_obstacle_angle < len(self.laser_scan_data.ranges) / 2:
                    twist_msg.angular.z = 0.5  # Turn right
                else:
                    twist_msg.angular.z = -0.5  # Turn left
            else:
                print('Stop')
                twist_msg.linear.x = 0.0  # Stop

            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_avoidance_node = ObstacleAvoidanceNode()
        obstacle_avoidance_node.run()
    except rospy.ROSInterruptException:
        pass