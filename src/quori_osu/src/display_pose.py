#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class PoseDisplay:
    def __init__(self):
        rospy.init_node('pose_display')
        self.image_sub = rospy.Subscriber('/astra_ros/devices/default/color/image_color', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/pose_display/image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.pose = mp.solutions.pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image and draw pose landmarks
        results = self.pose.process(cv_image)
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                image = cv_image,
                landmark_list = results.pose_landmarks,
                connections = mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())

        # Convert OpenCV image back to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # Publish the processed image
        self.image_pub.publish(ros_image)


if __name__ == "__main__":
    node = PoseDisplay()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
