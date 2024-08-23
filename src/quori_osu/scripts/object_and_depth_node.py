#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import tf
from visualization_msgs.msg import Marker
from threading import Timer
import threading
import math
from geometry_msgs.msg import PointStamped, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

class HumanDetectionWithDepth():
    def __init__(self):
        self.bridge = CvBridge()
        
        # Depth camera parameters
        self.FX_DEPTH = 577.3614501953125
        self.FY_DEPTH = 577.3614501953125
        self.CX_DEPTH = 322.93994140625
        self.CY_DEPTH = 237.65728759765625
        self.marker_id = 0
        #self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.coord_pub = rospy.Publisher('/human_coordinates', PointStamped, queue_size=10)

        # Load pre-trained human detection model (e.g., YOLO)
        yolo_cfg_path = '/opt/quori/src/quori_osu/scripts/yolov3.cfg'
        yolo_weights_path = '/opt/quori/src/quori_osu/scripts/yolov3.weights'
        self.net = cv2.dnn.readNet(yolo_weights_path, yolo_cfg_path)
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers().flatten()]

        self.classes = ["person"]

        # Subscribe to depth image topic and RGB image topic
        self.image_depth_sub = message_filters.Subscriber("/astra_ros/devices/default/depth/image", Image)
        self.image_rgb_sub = message_filters.Subscriber("/astra_ros/devices/default/color/image_color", Image)


        # Initialize TransformListener
        self.listener = tf.TransformListener()
        # Synchronize the depth and RGB image topics
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_depth_sub, self.image_rgb_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback_sync)


        # Synchronize image messages
        sync = message_filters.ApproximateTimeSynchronizer([self.image_depth_sub, self.image_rgb_sub], queue_size=10, slop=0.4)
        sync.registerCallback(self.callback_sync)
        rospy.loginfo('Node ready.')

    def callback_sync(self, depth_img_msg, rgb_img_msg):
        #rospy.loginfo(f"Received image - Frame ID: {depth_img_msg.header.frame_id}")
        #rospy.loginfo('Callback triggered.')
        try:
            # Convert ROS Image messages to OpenCV images
            depth_image = self.bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding='16UC1')
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding='bgr8')
            
            # Perform human detection
            bounding_boxes = self.detect_humans(rgb_image)
            coord_list = []

            for bbox in bounding_boxes:
                x, y, w, h = bbox        
                center_x = x + w // 2
                center_y = y + h // 2
                depth_value = depth_image[center_y, center_x]

                # Check if depth_value is reasonable
                if depth_value < 0 or depth_value > 10000:  # Example range
                    rospy.logwarn(f"Unreasonable depth value: {depth_value}")
                    continue

                coordinate_estimation = self.compute_coordinates(center_x, center_y, depth_image, depth_img_msg.header)
                if coordinate_estimation:
                    coord_list.append(coordinate_estimation)
                    # Publish coordinates
                    self.coord_pub.publish(coordinate_estimation)

                    # Draw bounding box and detected point (Green color)
                    cv2.rectangle(rgb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(rgb_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # Label depth value
                    cv2.putText(rgb_image, f"Depth: {depth_value}mm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(rgb_image, f"Depth: {depth_value}mm", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Label coordinates
                    transformed_x = coordinate_estimation.point.x
                    transformed_y = coordinate_estimation.point.y
                    #rospy.loginfo(f"Transformed coordinates: x={transformed_x}, y={transformed_y}")
                    transformed_z = coordinate_estimation.point.z
                    label = f"Coord: z={transformed_z:.2f}"
                    cv2.putText(rgb_image, label, (center_x, center_y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)  # Yellow color

            # Draw and publish origin marker
            '''
            origin_marker = Marker()
            origin_marker.header.frame_id = 'quori/base_link'
            origin_marker.header.stamp = rospy.Time.now()
            origin_marker.ns = 'human_detection'
            origin_marker.id = -1
            origin_marker.type = Marker.SPHERE
            origin_marker.action = Marker.ADD
            origin_marker.pose.position.x = 0
            origin_marker.pose.position.y = 0
            origin_marker.pose.position.z = 0
            origin_marker.scale.x = 0.1
            origin_marker.scale.y = 0.1
            origin_marker.scale.z = 0.1
            origin_marker.color.a = 1.0
            origin_marker.color.b = 1.0
            self.marker_pub.publish(origin_marker)'''

            # Display the result (optional, for debugging)
            if not rospy.is_shutdown():
                cv2.imshow("Human Detection with Depth", rgb_image)
                cv2.waitKey(1)

           # rospy.loginfo(f"Coordinates list: {coord_list}")      

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred: {e}")

    def detect_humans(self, image):
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        
        class_ids = []
        confidences = []
        boxes = []
        
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.3 and class_id < len(self.classes):
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        bounding_boxes = [boxes[i] for i in indexes.flatten()]
        return bounding_boxes
    

    def compute_coordinates(self, bbox_x_center, bbox_y_center, cv_image, header):
        try:
            # Convert depth value from mm to meters
            depth_value = float(cv_image[bbox_y_center, bbox_x_center])
            #rospy.loginfo(f"Depth value at center ({bbox_x_center}, {bbox_y_center}): {depth_value}")

            if depth_value == 0:  # Avoid division by zero
                rospy.logwarn("Depth value is zero; skipping computation.")
                return None
            
            # Create a PointStamped object for the point in the camera frame
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header = header
            point_in_camera_frame.header.frame_id = 'quori/head_camera'
            point_in_camera_frame.point.x = (bbox_x_center+150 - self.CX_DEPTH) / self.FX_DEPTH * depth_value #ADDED 150 to account for the offset of the camera
            point_in_camera_frame.point.y = (bbox_y_center - self.CY_DEPTH) / self.FY_DEPTH * depth_value
            point_in_camera_frame.point.z = depth_value

            # Transform the point to the map frame
            point_in_map_frame = self.listener.transformPoint('/map', point_in_camera_frame)
            x1 = point_in_map_frame.point.x
            y1 = point_in_map_frame.point.y
            z1 = point_in_map_frame.point.z

            #rospy.loginfo(f"Computed coordinates: x={x1}, y={y1}, z={z1}")

            # Create and publish Marker
            '''
            marker = Marker(
                header=Header(stamp=rospy.Time.now(), frame_id='map'),
                ns='human_detection',
                id=self.marker_id,
                type=Marker.SPHERE,
                action=Marker.ADD,
                pose=Pose(position=Point(x=x1, y=y1, z=z1)),
                color=ColorRGBA(a=1.0, r=1.0, g=0.0, b=0.0),
            )

            self.marker_id += 1
            self.marker_pub.publish(marker)
            '''
            self.coord_pub.publish(point_in_map_frame)
            
            #rospy.loginfo(f"Published point: {point_in_map_frame}")

            return point_in_map_frame
        except Exception as e:
            rospy.logerr(f"An error occurred while computing coordinates: {e}")
        return None

    def shutdown():
        print("Object detection shutting down...")
        rospy.signal_shutdown("shutting down after timeout")
    timer = threading.Timer(10.0, shutdown)
    timer.start()



if __name__=="__main__":
    rospy.init_node('human_detection_with_depth', anonymous=True)
    rospy.loginfo('Node started')

    HumanDetectionWithDepth()
    rospy.Timer(rospy.Duration(10), lambda event: rospy.signal_shutdown("shutdown after 10 seconds"))
    while not rospy.is_shutdown():
        rospy.spin()
    
