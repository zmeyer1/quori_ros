#!/usr/bin/env python3

import rospy
import actionlib
from rospy.timer import sleep
import tf
import tf.transformations as tf_trans
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import math

# Maximum speed for the controller
MAX_SPEED = 0.6
MIN_SPEED = 0.1

# Proportional gain for the controller
k_p_linear = 0.4
k_p_angular = 0.4

# Tolerance for yaw alignment (in radians, 1 degree ~ 0.017 rad)
# YAW_TOLERANCE = 0.0349  # ~2 degrees
YAW_TOLERANCE = 0.05  # ~3 degrees

# Distance tolerance for reaching the goal
DISTANCE_TOLLERANCE = 0.1

class DriveAndHomeQuoriNode:

    def __init__(self):
        rospy.init_node('drive_and_home_quori', anonymous=True)

        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/quori/base_controller/cmd_vel', Twist, queue_size=10)

        self.listener = tf.TransformListener()

        self.action_server = actionlib.SimpleActionServer('drive_to_point', MoveBaseAction, self.execute_drive, False)
        self.action_server.start()

        self.service = rospy.Service('/home_quori', Empty, self.home_quori_body)

        # Align the orientation of the body with the wheel axle initially
        # Doesnt seem to trigger everytime
        # self.home_quori_body(None)

        rospy.loginfo("Drive and Home Quori Node ready")

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def home_quori_body(self, req):
        rate = rospy.Rate(10)  # 10 Hz
        try:
            while not rospy.is_shutdown():
                # Wait for the transform to be available
                self.listener.waitForTransform('/quori/body_upper', '/ramsis/wheel_axle', rospy.Time(0), rospy.Duration(4.0))

                # Get the transformation between the frames
                (trans, rot) = self.listener.lookupTransform('/quori/body_upper', '/ramsis/wheel_axle', rospy.Time(0))

                # We only need the rotation part to align the orientation
                euler = tf_trans.euler_from_quaternion(rot)
                yaw = euler[2]  # Extract yaw from the quaternion (roll, pitch, yaw)

                # Calculate the current yaw
                current_yaw = tf_trans.euler_from_quaternion(self.listener.lookupTransform('/map', '/quori/base_link', rospy.Time(0))[1])[2]

                # Desired yaw to face forward (0 radians)
                desired_yaw = 0

                # Determine the error between the current yaw and desired yaw
                error_yaw = desired_yaw - current_yaw

                # # Calculate the yaw error
                # error_yaw = yaw - current_yaw

                # Normalize the yaw error to the range [-pi, pi]
                error_yaw = self.normalize_angle(error_yaw)

                # Check if the yaw error is within the acceptable tolerance
                if abs(error_yaw) < YAW_TOLERANCE:
                    rospy.loginfo(f"Alignment complete: Yaw Error = {error_yaw:.3f} rad (within tolerance)")
                    twist = Twist()
                    twist.angular.z = 0
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    self.cmd_vel_pub.publish(twist)  # Stop any motion
                    break

                # Create a Twist message to send the orientation command
                twist = Twist()
                twist.angular.z = k_p_angular * error_yaw
                twist.angular.z = max(min(twist.angular.z, MAX_SPEED), -MAX_SPEED)  # Clamp to max speed

                # Publish the command
                self.cmd_vel_pub.publish(twist)

                rospy.loginfo(f"Aligning /quori/body_upper with /ramsis/wheel_axle: Yaw Error = {error_yaw:.3f} rad")

                rate.sleep()

            return EmptyResponse()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform lookup failed: %s", e)
            return EmptyResponse()

    def execute_drive(self, goal):
        target_x = goal.target_pose.pose.position.x
        target_y = goal.target_pose.pose.position.y

        rospy.loginfo(f"Received goal: ({target_x}, {target_y})")

        marker = Marker()
        marker.header.frame_id = "map"
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

        rate = rospy.Rate(10.0)
        feedback = MoveBaseFeedback()
        twist = Twist()

        while not rospy.is_shutdown():
            try:
                self.marker_pub.publish(marker)

                (trans, rot) = self.listener.lookupTransform('/map', '/quori/base_link', rospy.Time(0))
                current_x, current_y = trans[0], trans[1]

                error_x = target_x - current_x
                error_y = target_y - current_y
                distance = np.hypot(error_x, error_y)

                if distance < DISTANCE_TOLLERANCE:
                    rospy.loginfo("Reached the goal!")

                    # Align the orientation of the body with the wheel axle
                    # self.home_quori_body(None)

                    # Stop the robot
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.cmd_vel_pub.publish(twist)
                    result = MoveBaseResult()
                    self.action_server.set_succeeded(result)
                    break

                desired_yaw = np.arctan2(error_y, error_x)
                current_yaw = tf_trans.euler_from_quaternion(rot)[2]
                error_yaw = desired_yaw - current_yaw
                error_yaw = np.arctan2(np.sin(error_yaw), np.cos(error_yaw))

                twist.angular.z = k_p_angular * error_yaw
                twist.angular.z = min(max(twist.angular.z, -MAX_SPEED), MAX_SPEED)

                # if abs(error_yaw) < 0.1:
                #     twist.linear.x = k_p_linear * distance
                #     twist.linear.x = min(twist.linear.x, MAX_SPEED)
                # else:
                #     twist.linear.x = 0

                if abs(error_yaw) < 0.1:
                    if distance > 0.5:
                        twist.linear.x = MAX_SPEED * np.log1p(distance - 1)  # Logarithmic ramp-up
                        twist.linear.x = min(twist.linear.x, MAX_SPEED)
                        twist.linear.x = max(twist.linear.x, MIN_SPEED)
                    else:
                        twist.linear.x = k_p_linear * distance
                        twist.linear.x = min(twist.linear.x, MAX_SPEED)
                        twist.linear.x = max(twist.linear.x, MIN_SPEED)
                else:
                    twist.linear.x = 0

                rospy.loginfo(f"Moving to: ({current_x}, {current_y})")
                rospy.logerr(f"Speed: {twist.linear.x}")
                self.cmd_vel_pub.publish(twist)

                feedback.base_position.pose.position.x = current_x
                feedback.base_position.pose.position.y = current_y
                self.action_server.publish_feedback(feedback)

                rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    # def execute_drive(self, goal):
    #     target_x = goal.target_pose.pose.position.x
    #     target_y = goal.target_pose.pose.position.y

    #     rospy.loginfo(f"Received goal: ({target_x}, {target_y})")

    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.type = marker.SPHERE
    #     marker.action = marker.ADD
    #     marker.pose.position.x = target_x
    #     marker.pose.position.y = target_y
    #     marker.pose.position.z = 0
    #     marker.pose.orientation.w = 1.0
    #     marker.scale.x = 0.2
    #     marker.scale.y = 0.2
    #     marker.scale.z = 0.2
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0

    #     rate = rospy.Rate(10.0)
    #     feedback = MoveBaseFeedback()
    #     twist = Twist()

    #     while not rospy.is_shutdown():
    #         try:
    #             rospy.loginfo("Publishing marker")
    #             self.marker_pub.publish(marker)

    #             (trans, rot) = self.listener.lookupTransform('/map', '/quori/base_link', rospy.Time(0))
    #             current_x, current_y = trans[0], trans[1]

    #             error_x = target_x - current_x
    #             error_y = target_y - current_y
    #             distance = np.hypot(error_x, error_y)

    #             if distance < 0.1:
    #                 rospy.loginfo("Reached the goal!")

    #                 # Stop the robot
    #                 twist.linear.x = 0
    #                 twist.angular.z = 0
    #                 self.cmd_vel_pub.publish(twist)
    #                 result = MoveBaseResult()
    #                 self.action_server.set_succeeded(result)
    #                 break

    #             desired_yaw = np.arctan2(error_y, error_x)
    #             current_yaw = tf_trans.euler_from_quaternion(rot)[2]
    #             error_yaw = desired_yaw - current_yaw
    #             error_yaw = np.arctan2(np.sin(error_yaw), np.cos(error_yaw))

    #             twist.angular.z = k_p_angular * error_yaw
    #             twist.angular.z = min(max(twist.angular.z, -MAX_SPEED), MAX_SPEED)

    #             if abs(error_yaw) < 0.1:
    #                 if distance > 1.0:
    #                     twist.linear.x = MAX_SPEED * np.log1p(distance - 1)  # Logarithmic ramp-up
    #                     twist.linear.x = min(twist.linear.x, MAX_SPEED)
    #                 else:
    #                     twist.linear.x = k_p_linear * distance
    #                     twist.linear.x = min(twist.linear.x, MAX_SPEED)
    #             else:
    #                 twist.linear.x = 0

    #             rospy.logdebug(f"Moving to: ({current_x}, {current_y} at yaw: {current_yaw:.3f} rad and {twist.linear.x:.3f} m/s)")
    #             self.cmd_vel_pub.publish(twist)

    #             feedback.base_position.pose.position.x = current_x
    #             feedback.base_position.pose.position.y = current_y
    #             self.action_server.publish_feedback(feedback)

    #             rate.sleep()

    #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #             continue


if __name__ == '__main__':
    try:
        DriveAndHomeQuoriNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
