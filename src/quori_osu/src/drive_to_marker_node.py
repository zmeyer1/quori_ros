#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Twist
import numpy as np
from collections import deque

class DriveToMarkerNode:

    def __init__(self):
        rospy.init_node('drive_to_marker', anonymous=True)
        
        self.cluster_centers_sub = rospy.Subscriber('/cluster_centers', PointStamped, self.cluster_centers_callback)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/quori/base_controller/cmd_vel', Twist, queue_size=10)
        
        self.cluster_centers = deque(maxlen=10)
        self.rate = rospy.Rate(1)  # 1 Hz

    def cluster_centers_callback(self, msg):
        center = [msg.point.x, msg.point.y]
        self.cluster_centers.append(center)
        
        if len(self.cluster_centers) == 10:
            self.publish_averaged_marker()

    def publish_averaged_marker(self):
        all_centers = np.array(self.cluster_centers)
        averaged_center = np.mean(all_centers, axis=0)

        # Assuming the robot's position is at the origin (0,0)
        robot_position = np.array([0, 0])

        # Calculate the direction vector from the object to the robot
        direction_vector = robot_position - averaged_center
        # Normalize the direction vector
        direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)
        # Calculate the new position 0.5 meters towards the robot from the averaged center
        new_position = averaged_center + 0.5 * direction_vector_normalized
        
        marker = Marker()
        marker.header.frame_id = "ramsis/base_laser_scanner"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "averaged_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = new_position[0]
        marker.pose.position.y = new_position[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        rospy.loginfo("Publishing averaged marker at: (%f, %f)", new_position[0], new_position[1])
        self.marker_pub.publish(marker)

        # Drive to the averaged marker position
        self.drive_to_position(new_position)

    def drive_to_position(self, target_position):
        # Assuming the robot's position is at the origin (0,0) for simplicity
        robot_position = np.array([0, 0])
        target_position = np.array(target_position)  # Convert target_position to numpy array if not already

        # Calculate distance and angle to the target
        distance = np.linalg.norm(target_position - robot_position)
        angle_to_target = np.arctan2(target_position[1] - robot_position[1], target_position[0] - robot_position[0])

        # Proportional control parameters
        angular_speed_factor = 0.1  # Adjust as necessary
        linear_speed_factor = 0.1  # Adjust as necessary

        # Create Twist message
        command = Twist()
        command.angular.z = angle_to_target * angular_speed_factor
        command.linear.x = min(distance, 1.0) * linear_speed_factor  # Limit max speed

        # Publish command
        self.cmd_vel_pub.publish(command)

    # def drive_to(self, position):
    #     target_x, target_y = position
    #     distance_threshold = 0.5  # Stop within X meters of the target
    #     angle_threshold = 0.1  # Angle threshold to stop turning (radians)

    #     while not rospy.is_shutdown():
    #         # Calculate distance and angle to the target
    #         distance = np.hypot(target_x, target_y)
    #         angle = np.arctan2(target_y, target_x)

    #         if distance < distance_threshold:
    #             # If within the distance threshold, stop the robot
    #             rospy.loginfo("Reached the target position.")
    #             self.stop_robot()
    #             break

    #         # Create a Twist message
    #         twist = Twist()

    #         if abs(angle) > angle_threshold:
    #             # Rotate towards the target
    #             twist.angular.z = 0.3 * angle / abs(angle)  # Adjust rotation speed
    #             twist.linear.x = 0.0
    #         else:
    #             # Move towards the target
    #             twist.linear.x = min(0.5, distance)  # Adjust forward speed
    #             twist.angular.z = 0.0

    #         # Publish the Twist message
    #         self.cmd_vel_pub.publish(twist)

    #         # Sleep for a short duration to allow the robot to move
    #         rospy.sleep(0.1)

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)
        

if __name__ == '__main__':
    node = DriveToMarkerNode()
    rospy.spin()
