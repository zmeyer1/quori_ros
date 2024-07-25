#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import tf2_ros

class MarkerPublisher:

    def __init__(self):
        rospy.init_node('marker_publisher_node', anonymous=True)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(1)  # 1 Hz

    def create_marker(self, x, y, id):
        # Function to create a marker
        marker = Marker()
        marker.header.frame_id = "map"  # Markers should be in the map frame
        marker.header.stamp = rospy.Time.now()

        marker.ns = "markers"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2  # size of the marker
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()

        return marker

    def transform_points(self, points, source_frame, target_frame):
        # Function to transform points from source_frame to target_frame
        transformed_points = []
        for point in points:
            if source_frame == "ramsis/base_laser_scanner" and target_frame == "map":
                # Handle 90-degree rotation
                transformed_points.append([point[1], -point[0]])  # Rotate 90 degrees counter clockwise
            elif source_frame == "map" and target_frame == "ramsis/base_laser_scanner":
                # Handle inverse rotation
                transformed_points.append([-point[1], point[0]])  # Rotate 90 degrees clockwise
            else:
                rospy.logerr("Unsupported transformation from {} to {}".format(source_frame, target_frame))
                return []

        return np.array(transformed_points)

    def publish_markers(self):
        # Function to publish markers
        while not rospy.is_shutdown():
            points = np.array([[-1, 1], [1, 1]])  # Example points in ramsis/base_laser_scanner frame

            # Transform points from ramsis/base_laser_scanner to map
            transformed_points = self.transform_points(points, "ramsis/base_laser_scanner", "map")

            if len(transformed_points) > 0:
                marker_array = MarkerArray()
                for i, point in enumerate(transformed_points):
                    marker = self.create_marker(point[0], point[1], i)
                    marker_array.markers.append(marker)

                rospy.loginfo("Publishing markers at %s", transformed_points)
                self.marker_pub.publish(marker_array)
            else:
                rospy.logwarn("No points to publish")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = MarkerPublisher()
        node.publish_markers()
    except rospy.ROSInterruptException:
        pass



# #!/usr/bin/env python3

# import rospy
# import numpy as np
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import PointStamped, TransformStamped
# import tf2_ros
# import tf2_geometry_msgs

# class MarkerPublisher:

#     def __init__(self):
#         rospy.init_node('marker_publisher_node', anonymous=True)
#         self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#         self.rate = rospy.Rate(1)  # 1 Hz

#     def create_marker(self, x, y, id):
#         marker = Marker()
#         marker.header.frame_id = "ramsis/base_laser_scanner"  
#         marker.header.stamp = rospy.Time.now()

#         marker.ns = "markers"
#         marker.id = id
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD

#         marker.pose.position.x = x
#         marker.pose.position.y = y
#         marker.pose.position.z = 0

#         marker.pose.orientation.x = 0.0
#         marker.pose.orientation.y = 0.0
#         marker.pose.orientation.z = 0.0
#         marker.pose.orientation.w = 1.0

#         marker.scale.x = 0.2  # size of the marker
#         marker.scale.y = 0.2
#         marker.scale.z = 0.2

#         marker.color.r = 0.0
#         marker.color.g = 0.0
#         marker.color.b = 1.0
#         marker.color.a = 1.0

#         marker.lifetime = rospy.Duration()

#         return marker

#     def transform_points(self, points, source_frame, target_frame):
#         fixed_frame_points = []
#         for point in points:
#             point_stamped = PointStamped()
#             point_stamped.header.frame_id = source_frame
#             point_stamped.header.stamp = rospy.Time.now()
#             point_stamped.point.x = point[0]
#             point_stamped.point.y = point[1]
#             point_stamped.point.z = 0.0

#             try:
#                 transformed_point = self.tf_buffer.transform(point_stamped, target_frame, rospy.Duration(1.0))
#                 fixed_frame_points.append([transformed_point.point.x, transformed_point.point.y])
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#                 rospy.logerr("Failed to transform point: %s" % str(e))
#                 return []

#         return np.array(fixed_frame_points)

#     def publish_markers(self):
#         while not rospy.is_shutdown():
#             points = np.array([[-1, 1], [1, 1]])  
#             # fixed_frame_points = self.transform_points(points, "ramsis/base_laser_scanner", "map")
            
#             # if len(fixed_frame_points) > 0:
#             #     marker_array = MarkerArray()
#             #     for i, point in enumerate(fixed_frame_points):
#             #         marker = self.create_marker(point[0], point[1], i)
#             #         marker_array.markers.append(marker)

#             #     rospy.loginfo("Publishing markers at %s", fixed_frame_points)

#             #     self.marker_pub.publish(marker_array)
#             # else:
#             #     rospy.logwarn("No points to publish")

#             # points = self.transform_points(points, "ramsis/base_laser_scanner", "map")
#             if len(points) > 0:
#                 marker_array = MarkerArray()
#                 for i, point in enumerate(points):
#                     marker = self.create_marker(point[0], point[1], i)
#                     marker_array.markers.append(marker)

#                 rospy.loginfo("Publishing markers at %s", points)

#                 self.marker_pub.publish(marker_array)
#             else:
#                 rospy.logwarn("No points to publish")

#             self.rate.sleep()

# if __name__ == '__main__':
#     try:
#         node = MarkerPublisher()
#         node.publish_markers()
#     except rospy.ROSInterruptException:
#         pass


# #!/usr/bin/env python3

# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point

# def create_marker(x, y, id):
#     marker = Marker()
#     marker.header.frame_id = "ramsis/base_laser_scanner"  # assuming markers are in robot's frame
#     marker.header.stamp = rospy.Time.now()

#     marker.ns = "markers"
#     marker.id = id
#     marker.type = Marker.SPHERE
#     marker.action = Marker.ADD

#     marker.pose.position.x = x
#     marker.pose.position.y = y
#     marker.pose.position.z = 0

#     marker.pose.orientation.x = 0.0
#     marker.pose.orientation.y = 0.0
#     marker.pose.orientation.z = 0.0
#     marker.pose.orientation.w = 1.0

#     marker.scale.x = 0.2  # size of the marker
#     marker.scale.y = 0.2
#     marker.scale.z = 0.2

#     marker.color.r = 0.0
#     marker.color.g = 0.0
#     marker.color.b = 1.0
#     marker.color.a = 1.0

#     marker.lifetime = rospy.Duration()

#     return marker

# def marker_publisher():
#     rospy.init_node('marker_publisher_node', anonymous=True)
#     marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
#     tf_buffer = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#     rate = rospy.Rate(1)  # 1 Hz

#     while not rospy.is_shutdown():
#         marker1 = create_marker(-1, 1, 0)
#         marker2 = create_marker(1, 1, 1)

#         marker_pub.publish(marker1)
#         marker_pub.publish(marker2)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         marker_publisher()
#     except rospy.ROSInterruptException:
#         pass

