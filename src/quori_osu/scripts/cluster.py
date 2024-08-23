#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped
from threading import Timer
import threading
import time
import math
import random

data = []
class Cluster:

    def __init__(self, pixel_size=0.01, dilation_size=None):
        """
        Initialize the Cluster class with specified pixel size and dilation size.
        """
        self.pixel_size = pixel_size
        self.dilation_size = dilation_size
        self.human_pose = []

    def add_data(self, point):
        """
        Add a new data point to the cluster.
        """
        data.append(point)

    def fit(self):
        """
        Process the data points in the cluster, normalize, adjust, and apply image processing techniques.
        """
        if not data:
            print("No data to process")
            return

        # Normalize the data based on pixel size
        new_data = np.array([(int(x / self.pixel_size), int(y / self.pixel_size)) for x, y in data])
        #print(f"Normalized data: {new_data}")

        # Adjust the data to start from (0,0)
        min_x, min_y = np.min(new_data[:, 0]), np.min(new_data[:, 1])
        new_data[:, 0] -= min_x
        new_data[:, 1] -= min_y
        #print(f"Adjusted data: {new_data}")

        # Determine the dimensions of the image
        max_x, max_y = np.max(new_data[:, 0]), np.max(new_data[:, 1])
        #print(f"Image Dimensions: ({max_x + 1}, {max_y + 1})")

        # Create an empty image and plot the points
        image = np.zeros((max_x + 1, max_y + 1), dtype=np.uint8)
        for x, y in new_data:
            if 0 <= x < max_x and 0 <= y < max_y:
                image[x, y] = 255

        #print(f"Image data:\n{image}")

        # Save the initial image
        image_path = '/home/quori6/initial_image.png'
        plt.imsave(image_path, image, cmap='gray')
        #print(f"Saved initial image to {image_path}")

        # Apply dilation and erosion if dilation_size is specified
        if self.dilation_size:
            kernel = np.ones((self.dilation_size, self.dilation_size), np.uint8)
            image = cv.dilate(image, kernel, iterations=1)
            image = cv.erode(image, kernel, iterations=1)

        # Save the dilated/eroded image
        image_path = '/home/quori6/dilated_eroded_image.png'
        plt.imsave(image_path, image, cmap='gray')

        # Use connected components to label the image regions
        n_labels, labels, stats, centroids = cv.connectedComponentsWithStats(image, 8, cv.CV_8U)
        print(f'{n_labels} total regions')
        #print(f'Stats: {stats}')

        condition = np.logical_and(stats[:, 4] > 3, stats[:, 4] < 1000)
        self.human_pose = []
        for i in range(1, n_labels):
            if condition[i] == True:
                cx, cy = centroids[i]
                l = [(cx+min_x)*self.pixel_size, (cy+min_y)*self.pixel_size, 0]
                self.human_pose.append(l)
                print(f'Component {i}: Centroid at ({(cx+min_x)*self.pixel_size:.2f}, {(cy+min_y)*self.pixel_size:.2f})')
                print(self.human_pose)

        # Count the number of objects based on the size of the regions
        num_objects = sum(np.logical_and(stats[:, 4] > 3, stats[:, 4] < 1000))
        print(f'{num_objects} actual objects')

        # Save the labeled image
        labeled_image_path = '/home/quori6/labeled_image.png'
        plt.imsave(labeled_image_path, labels, cmap='nipy_spectral')

class CoordinateClusterNode:
    def __init__(self):
        """
        Initialize the CoordinateClusterNode class, set up the ROS node, subscriber, and timer.
        """
        rospy.init_node('coordinate_cluster_node')
        self.sub = rospy.Subscriber('/human_coordinates', PointStamped, self.callback)
        #self.pub = rospy.Publisher('/visualization_marker', PointStamped, queue_size = 10)
        self.publisher = rospy.Publisher('/last_two_coordinates', PointStamped, queue_size=10)
        self.cluster = Cluster(pixel_size=0.01, dilation_size=20)
        self.coordinates = []
        rospy.loginfo("CoordinateClusterNode initialized")

    def callback(self, msg):
        """
        Callback function for the subscriber to process incoming messages.
        """
        x = msg.point.x
        y = msg.point.y
        #rospy.loginfo(f"Received coordinates: x={x}, y={y}")
        # Add received coordinates to the cluster
        self.cluster.add_data((x, y))
        #rospy.loginfo("Data added to cluster")
        # Process the data (for example, fit clusters and filter)
        self.cluster.fit()
        #rospy.loginfo("Cluster fitting completed")
        

    def process_data(self):
        """
        Process the collected data points by adding them to the cluster and fitting the cluster.
        """
        # Add all collected coordinates to the cluster and fit it
        for coord in self.coordinates:
            self.cluster.add_data(coord)
        self.cluster.fit()

        # Clear the coordinates list
        self.coordinates.clear()

    def distance(self, p1, p2):
       """Calculate the Euclidean distance between two points."""
       return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def shutdown(self):
        print("Cluster shutting down...")
        human_coordinates = self.cluster.human_pose
        min_distance = 100.00
        closest_pair = (None, None)
        if len(human_coordinates) > 2:
           for i in range(len(human_coordinates)):
                for j in range(i + 1, len(human_coordinates)):
                    if i != j:
                        d = self.distance(human_coordinates[i], human_coordinates[j])
                        if d < min_distance:
                            min_distance = d
                            closest_pair = (human_coordinates[i], human_coordinates[j])
        else:
           min_distance  = self.distance(human_coordinates[0], human_coordinates[1])
           closest_pair = (human_coordinates[0], human_coordinates[1])
        mid_point  = ((closest_pair[0][0]+closest_pair[1][0])/2, (closest_pair[0][1]+closest_pair[1][1])/2)
        print(f"The distance between the closest pair is : {min_distance}")
        # Generate a random integer between 1 and 5 (inclusive 
        random_integer = random.randint(1, 5)
        if closest_pair[0][1]-closest_pair[1][1]>0 and closest_pair[0][0]-closest_pair[1][0]>0:
            quori_target_xy = (mid_point[0]+random_integer*abs(closest_pair[0][1]-closest_pair[1][1]), mid_point[1]-random_integer*abs(closest_pair[0][0]-closest_pair[1][0]))
        elif closest_pair[0][1]-closest_pair[1][1]<0 and closest_pair[0][0]-closest_pair[1][0]<0:
            quori_target_xy = (mid_point[0]+random_integer*abs(closest_pair[0][1]-closest_pair[1][1]), mid_point[1]-random_integer*abs(closest_pair[0][0]-closest_pair[1][0]))
        else:
            quori_target_xy = (mid_point[0]-random_integer*abs(closest_pair[0][1]-closest_pair[1][1]), mid_point[1]-random_integer*abs(closest_pair[0][0]-closest_pair[1][0]))
        print(f"quori_target_xy: {quori_target_xy}")
        # Create the PointStamped message
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "quori/head_camera"
        point_msg.point.x, point_msg.point.y, point_msg.point.z = point
        rospy.loginfo(f"Publishing coordinate: {point_msg}")
        self.publisher.publish(point_msg)
        time.sleep(5)
        rospy.signal_shutdown("shutting down after timeout")


obj = CoordinateClusterNode()
timer = threading.Timer(11.0, obj.shutdown)
timer.start()
if __name__ == '__main__':
    node = CoordinateClusterNode()
    rospy.spin()
