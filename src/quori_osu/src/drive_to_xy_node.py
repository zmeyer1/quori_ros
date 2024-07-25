#!/usr/bin/env python3

import rospy
import actionlib
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
import numpy as np
from collections import deque

class DriveToXYNode:

    def __init__(self):
        rospy.init_node('drive_to_xy', anonymous=True)
        
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/quori/base_controller/cmd_vel', Twist, queue_size=10)
        
        self.action_server = actionlib.SimpleActionServer('drive_to_point', MoveBaseAction, self.execute, False)
        self.action_server.start()

    def execute(self, goal):
        # Extract XY coordinates from the goal
        target_x = goal.target_pose.pose.position.x
        target_y = goal.target_pose.pose.position.y
        
        # Publish a marker at the target location
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = target_x
        marker.pose.position.y = target_y
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        

        # Assuming the robot's position is at the origin (0,0) for simplicity
        direction = np.array([target_x, target_y])
        distance = np.linalg.norm(direction)
        direction_normalized = direction / distance

        twist = Twist()
        feedback = MoveBaseFeedback()

        # Define a smaller velocity factor for smoother movement
        velocity_factor = 0.1  # Adjust this value as needed for smoother movement

        while distance > 0.1:
            # Apply a smaller velocity based on the direction towards the target
            twist.linear.x = velocity_factor * direction[0]
            twist.linear.y = velocity_factor * direction[1]
            self.cmd_vel_pub.publish(twist)
            
            # Simulate movement towards the target more accurately
            moved_distance = velocity_factor  # This assumes the robot moves this distance in each loop iteration
            distance -= moved_distance  # Update the remaining distance based on the movement
            
            # Update and publish feedback
            feedback.base_position.pose.position.x += twist.linear.x
            feedback.base_position.pose.position.y += twist.linear.y
            self.action_server.publish_feedback(feedback)
            
            rospy.sleep(0.1)  # Adjust sleep duration for smoother updates

        twist.linear.x = 0
        twist.linear.y = 0
        self.cmd_vel_pub.publish(twist)

        # Set the action result
        result = MoveBaseResult()
        self.action_server.set_succeeded(result)
        # # Assuming the robot's position is at the origin (0,0) for simplicity
        # direction = np.array([target_x, target_y])
        # distance = np.linalg.norm(direction)
        # direction_normalized = direction / distance
        
        # twist = Twist()
        # feedback = MoveBaseFeedback()
        # while distance > 0.1:
        #     # twist.linear.x = min(distance, 0.5) * direction_normalized[0]
        #     # twist.linear.y = min(distance, 0.5) * direction_normalized[1]
        #     twist.linear.x = target_x
        #     twist.linear.y = target_y
        #     self.cmd_vel_pub.publish(twist)
            
        #     # Update and publish feedback
        #     distance -= 0.5  # Simulate movement towards the target
        #     feedback.base_position.pose.position.x = target_x - distance * direction_normalized[0]
        #     feedback.base_position.pose.position.y = target_y - distance * direction_normalized[1]
        #     self.action_server.publish_feedback(feedback)
            
        #     rospy.sleep(1)
        
        # twist.linear.x = 0
        # twist.linear.y = 0
        # self.cmd_vel_pub.publish(twist)
        
        # # Set the action result
        # result = MoveBaseResult()
        # self.action_server.set_succeeded(result)

if __name__ == '__main__':
    try:
        DriveToXYNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass