#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sklearn.cluster import KMeans
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped

class LidarCluster:

    def __init__(self):
        rospy.init_node('lidar_cluster_node', anonymous=True)
        
        self.scan_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.center_pub = rospy.Publisher('cluster_centers', PointStamped, queue_size=10)
        
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # self.cluster_centers_accumulator = []
        # self.publish_average_timer = rospy.Timer(rospy.Duration(5), self.publish_average_cluster_center)
        
        self.k = 1

    def scan_callback(self, scan_data):

        ranges = np.array(scan_data.ranges)
        angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))
        
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
         
        points = np.vstack((x, y)).T
        
        if len(points) > 0:
            # Preprocess data to remove outliers
            points = self.preprocess_data(points)
            
            # KMeans initialization
            # initial_centers = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
            # initial_centers = np.array([[0.0, 0.0], [0.0, 0.0]])
            # kmeans = KMeans(n_clusters=self.k, init=initial_centers, n_init=1)
            # kmeans.fit(points)

            # Improved KMeans initialization
            kmeans = KMeans(
                n_clusters=self.k, 
                init='k-means++', 
                n_init=10, 
                max_iter=1000, 
            )
            kmeans.fit(points)
            cluster_centers = kmeans.cluster_centers_

            # rospy.loginfo("Cluster centers according to LIDAR: \n%s", cluster_centers)

            # Sort cluster centers by x-coordinate
            if(len(cluster_centers) == 2):
                if(cluster_centers[0][0] > cluster_centers[1][0]):
                    cluster_centers = np.array([cluster_centers[1], cluster_centers[0]])
                    # rospy.loginfo(f"Cluster centers: \n {cluster_centers[0]} and {cluster_centers[1]}") 
            
            # Create a PointStamped message for each cluster center
            for center in cluster_centers:
                point_stamped = PointStamped()
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.header.frame_id = scan_data.header.frame_id  # Assuming frame_id is available
                point_stamped.point = Point(center[0], center[1], 0.0)  # Z is set to 0.0 by default
                
                self.center_pub.publish(point_stamped)  

            # fixed_frame_centers = self.transform_points(cluster_centers, "map", scan_data.header.frame_id)
            # self.cluster_centers_accumulator.extend(fixed_frame_centers)
            # self.publish_cluster_marker(fixed_frame_centers)
            self.publish_cluster_marker(cluster_centers)


    def preprocess_data(self, points):
        # Filter out points that are too close or too far
        min_distance = 0.2
        max_distance = 7.0
        distances = np.linalg.norm(points, axis=1)
        filtered_points = points[(distances > min_distance) & (distances < max_distance)]
        
        return filtered_points
    

    def publish_cluster_marker(self, cluster_centers):
        marker_array = MarkerArray()

        for i, center in enumerate(cluster_centers):
            marker = Marker()
            marker.header.frame_id = "ramsis/base_laser_scanner"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "kmeans"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)
            
            # 20ms delay between markers, might need to be adjusted/deleted
            rospy.sleep(0.02)

        rospy.loginfo("Cluster centers according to MAP: \n%s", cluster_centers)
        self.marker_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LidarCluster()
    node.run()
