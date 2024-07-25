#!/usr/bin/env python3

import rospy
import os
import subprocess
import threading
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import messagebox
import queue

simple_question_list = ["Money", "Breathe", "Time", "Eclipse"]
simple_audio_list = ["Money.mp3", "Breathe.mp3", "Time.mp3", "Eclipse.mp3"]
simple_audio_path = "/home/quori6/Music/darksideofthemoon/"

current_joke_index = 0
current_process = None
# current_gui_root = None

# Create a queue to hold tasks for the tkinter main thread
task_queue = queue.Queue()

def play_audio(file_path):
    global current_process
    # Use mpg123 to play the audio file
    current_process = subprocess.Popen(["mpg123", file_path])

def stop_audio():
    global current_process
    if current_process:
        current_process.terminate()
        current_process = None
    # task_queue.put(lambda: close_gui())

# def show_question(index):
#     question = simple_question_list[index]
#     messagebox.showinfo("Question", question)

def play_next_clip():
    global current_joke_index
    rospy.loginfo("Playing clip %d...", current_joke_index + 1)
    folder_path = os.path.expanduser(simple_audio_path)

    if current_joke_index < len(simple_audio_list):
        file_name = simple_audio_list[current_joke_index]
        file_path = os.path.join(folder_path, file_name)
        if os.path.isfile(file_path):
            rospy.loginfo("Playing file: %s", file_path)
            # This is where we wan to publish the start service to the GUI
            # PUT CODE HERE
            #task_queue.put(lambda: show_question(current_joke_index))
            play_audio(file_path)
            current_joke_index += 1
        else:
            rospy.logwarn("File not found: %s", file_path)
    else:
        rospy.loginfo("No more clips to play.")

def joy_callback(data):
    rospy.loginfo(f"Buttons: '{data.buttons}'")
    if data.buttons == (0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0):
        task_queue.put(play_next_clip)
    elif data.buttons == (0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0):
        task_queue.put(stop_audio)

def listener():
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()

def process_tasks():
    try:
        while not task_queue.empty():
            task = task_queue.get_nowait()
            task()
    except queue.Empty:
        pass
    root.after(100, process_tasks)

# def close_gui():
#     global current_gui_root
#     if current_gui_root:
#         current_gui_root.destroy()
#         current_gui_root = None

if __name__ == '__main__':
    rospy.init_node('audio_player', anonymous=True)

    root = tk.Tk()
    root.withdraw()  # Hide the main root window

    # Start processing tasks from the queue
    root.after(100, process_tasks)

    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    root.mainloop()


