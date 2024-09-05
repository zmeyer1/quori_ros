#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from threading import Timer
import threading
import time
from visualization_msgs.msg import Marker
import math
import random

data = []
class Cluster:

    def __init__(self, pixel_size=0.001, dilation_size=20):
        """
        Initialize t
         Cluster class with specified pixel size and dilation size.
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
        #print(f"Data: {data}")

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

        # Create an empty image and plot the points
        image = np.zeros((max_x + 1, max_y + 1), dtype=np.uint8)
        for x, y in new_data:
            if 0 <= x < max_x and 0 <= y < max_y:
                image[x, y] = 255

        # Save the initial image
        image_path = '/home/quori6/initial_image.png'
        #plt.imsave(image_path, image, cmap='gray')
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

        condition = np.logical_and(stats[:, 4] > 3, stats[:, 4] < 100000)
        self.human_pose = []
        for i in range(1, n_labels):
            cy, cx = centroids[i]
            l = [(cx+min_x)*self.pixel_size, (cy+min_y)*self.pixel_size, 0]
            #print(f'Component {i}: Centroid at ({(cy+min_x)*self.pixel_size:.2f}, {(cx+min_y)*self.pixel_size:.2f})')

            if condition[i] == True:
                self.human_pose.append(l)
        print(self.human_pose)

        # Count the number of objects based on the size of the regions
        print(f'{sum(condition)} actual objects')

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
        import actionlib
        from move_base_msgs.msg import MoveBaseAction
        
        self.action_client = actionlib.SimpleActionClient('drive_to_point', MoveBaseAction)
        self.action_client.wait_for_server()


        self.cluster = Cluster(pixel_size=1, dilation_size=20)
        self.coordinates = []
        rospy.loginfo("CoordinateClusterNode initialized")

    def callback(self, msg):
        """
        Callback function for the subscriber to process incoming messages.
        """
        x = msg.point.z
        y = -msg.point.x
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
        if len(human_coordinates) < 2:
            rospy.logwarn("Less than 2 human coordinates")
            # Use return or handle the condition appropriately
            rospy.signal_shutdown("shutting down after timeout")
        else:
            min_distance = 1000000000000000
            closest_pair = (None, None)
            for i in range(len(human_coordinates)):
                for j in range(i + 1, len(human_coordinates)):
                    if i != j:
                        d = self.distance(human_coordinates[i], human_coordinates[j])
                        if (d < min_distance and d > 300): 
                            #do not consider the distance between the two humans if it is less than 300mm to aviod having one human count for twice
                            min_distance = d
                            closest_pair = (human_coordinates[i], human_coordinates[j])
            if closest_pair[0] is not None and closest_pair[1] is not None:
                mid_point  = ((closest_pair[0][0]+closest_pair[1][0])/2, (closest_pair[0][1]+closest_pair[1][1])/2)
                print(f"the closest pair is : {closest_pair}")
                print(f"the midpoint between two people is : {mid_point}")
                print(f"The distance between the closest pair is : {min_distance}")
                # Get the distance between quori goal pose and midpoint by generate a random integer between 100 and 1500 (inclusive)
                random123 = random.uniform(100, 1500)

                dx = abs(closest_pair[0][0] - closest_pair[1][0])
                dy = abs(closest_pair[0][1] - closest_pair[1][1])
                mx = dy/(math.sqrt(dx**2 + dy**2))*random123
                my = dx/(math.sqrt(dx**2 + dy**2))*random123
                print(my+mx)

                if (closest_pair[0][1]-closest_pair[1][1])>0 and (closest_pair[0][0]-closest_pair[1][0])>0:
                    quori_target_xy = (mid_point[0]-mx, mid_point[1]+my)
                elif closest_pair[0][1]-closest_pair[1][1]<0 and closest_pair[0][0]-closest_pair[1][0]<0:
                    quori_target_xy = (mid_point[0]-mx, mid_point[1]+my)
                else:
                    quori_target_xy = (mid_point[0]-mx, mid_point[1]-my)
                rospy.loginfo(f"random number : {random123}")
                rospy.logerr(f"Quori Goal Position: x = {quori_target_xy[0]/1000} , y = {quori_target_xy[1]/1000}")
                # Publish the target coordinates
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "base_link"  # or the appropriate frame
                goal.target_pose.header.stamp = rospy.Time.now()

                goal.target_pose.pose.position.x = quori_target_xy[0]/1000
                goal.target_pose.pose.position.y = quori_target_xy[1]/1000
                goal.target_pose.pose.position.z = 0.0

                # Assuming the robot should face forward in the direction of the target
                goal.target_pose.pose.orientation.w = 1.0  # No rotation

                if not rospy.is_shutdown():
                    print("sending goal")
                    self.action_client.send_goal(goal)
                else:
                    rospy.logwarn("Attempted to publish after shutdown")
            
            else:
                rospy.logwarn("No valid closest pair found")
                
            time.sleep(5)
            rospy.signal_shutdown("shutting down after timeout")
'''
                p1_marker = Marker()
                p1_marker.header.frame_id = 'quori/base_link'
                p1_marker.header.stamp = rospy.Time.now()
                p1_marker.ns = 'human_detection'
                p1_marker.id = -1
                p1_marker.type = Marker.SPHERE
                p1_marker.action = Marker.ADD
                p1_marker.pose.position.x = closest_pair[0][0]
                p1_marker.pose.position.y = closest_pair[0][1]
                p1_marker.pose.position.z = 0
                p1_marker.scale.x = 0.1
                p1_marker.scale.y = 0.1
                p1_marker.scale.z = 0.1
                p1_marker.color.a = 1.0
                p1_marker.color.b = 1.0
                self.marker_pub.publish(p1_marker)

                p2_marker = Marker()
                p2_marker.header.frame_id = 'quori/base_link'
                p2_marker.header.stamp = rospy.Time.now()
                p2_marker.ns = 'human_detection'
                p2_marker.id = -1
                p2_marker.type = Marker.SPHERE
                p2_marker.action = Marker.ADD
                p2_marker.pose.position.x = closest_pair[0][0]
                p2_marker.pose.position.y = closest_pair[0][1]
                p2_marker.pose.position.z = 0
                p2_marker.scale.x = 0.1
                p2_marker.scale.y = 0.1
                p2_marker.scale.z = 0.1
                p2_marker.color.a = 1.0
                p2_marker.color.b = 1.0
                self.marker_pub.publish(p2_marker)'''




if __name__ == '__main__':
    node = CoordinateClusterNode()
    timer = threading.Timer(11.0, node.shutdown)
    timer.start()
    rospy.spin()
