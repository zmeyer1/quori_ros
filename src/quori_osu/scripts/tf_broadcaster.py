#!/usr/bin/env python3

import rospy
import tf

def broadcaster():
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        br.sendTransform((1.0, 2.0, 3.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "child_frame",
                         "parent_frame")
        rate.sleep()


if __name__ == '__main__':
    broadcaster()
