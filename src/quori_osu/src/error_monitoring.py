#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Log
import subprocess
import time

LAUNCH_COMMAND = ["roslaunch", "quori_osu", "quori_proportional.launch"]
ERROR_MESSAGE = "Tried to publish before configured, topic id 125"

def callback(data):
    if ERROR_MESSAGE in data.msg:
        rospy.logerr("Error detected! Restarting roslaunch...")
        # subprocess.Popen(['rosnode', 'kill', '/error_monitoring'])  # Kill the current node to stop callback
        # time.sleep(5)  # Optional delay before restart
        # subprocess.Popen(LAUNCH_COMMAND)

def listener():
    rospy.init_node('error_monitoring', anonymous=True)
    rospy.Subscriber("/rosout_agg", Log, callback)
    # subprocess.Popen(LAUNCH_COMMAND)  # Start the launch file initially
    rospy.spin()

if __name__ == '__main__':
    listener()
