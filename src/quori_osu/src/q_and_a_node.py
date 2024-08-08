#!/usr/bin/env python3

import random
import rospy
import os
import subprocess
import threading
import signal
from sensor_msgs.msg import Joy
import tkinter as tk
import queue
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Int32

# Lists for questions and audio files

# Pink Floyd - Dark Side of the Moon Testing
# simple_question_list = ["Money", "Breathe", "Time", "Eclipse"]
# simple_audio_list = ["Money.mp3", "Breathe.mp3", "Time.mp3", "Eclipse.mp3"]
# simple_audio_path = "/home/quori6/Music/darksideofthemoon/"
# introduction_file = os.path.join(simple_audio_path, "Brain Damage.mp3")


# Simple Questions from Survey
simple_audio_path = "/home/quori6/Music/simple_questions/"
introduction_file = os.path.join(simple_audio_path, "SimpleIntro.mp3")
simple_question_list = [
    "What is the sum of 2+2?",
    "How many days are in a week?",
    "How many letters are in the English alphabet?",
    "How many minutes are in an hour?",
    "How many sides does a triangle have?",
    "What color is the sky",
    "What is the capital of France?",
    "What is the capital of Italy?",
    "What is the capital of the United States?",
    "What is the currency of Japan?",
    "What is the largest planet in our solar system?",
    "What is the square root of 16?"
    ]

simple_audio_list = [
    "TwoPlusTwo.mp3",
    "SevenDaysAWeek.mp3",
    "EnglishLetters.mp3",
    "SixtyMin.mp3",
    "Triangle.mp3",
    "BlueSky.mp3",
    "ParisCapital.mp3",
    "RomeCapital.mp3",
    "DCCapital.mp3",
    "Yen.mp3",
    "Jupiter.mp3",
    "SQRTFour.mp3"
    ]

simple_response_list = []
delay_times = [1, 1.5, 2, 2.5, 3]


# Indexes and counters
current_text_index = 0
current_audio_index = 0
next_button_count = 0
current_delay = 0
current_process = None
task_queue = queue.Queue()
introduction_played = False
gui_started = False

# Flag to stop the loop
all_questions_exhausted = False

def set_audio_output(sink_name):
    try:
        subprocess.run(['pactl', 'set-default-sink', sink_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to set audio output: {e}")

def init():
    # Set the audio output to "Headphones - Built In Audio"
    set_audio_output('alsa_output.pci-0000_00_1f.3.analog-stereo')

# Initialize the service clients
def init_service_clients():
    rospy.wait_for_service('start_gui')
    rospy.wait_for_service('stop_gui')
    start_gui_service = rospy.ServiceProxy('start_gui', Empty)
    stop_gui_service = rospy.ServiceProxy('stop_gui', Empty)
    return start_gui_service, stop_gui_service

# def next_value_callback(msg):
#     global current_text_index, simple_response_list
#     simple_response_list.append(msg.data)
#     rospy.loginfo(f"Appended {msg.data} to simple_response_list: {simple_response_list}")

#     # Update the current question index based on the received integer
#     if 0 <= msg.data < len(simple_question_list):
#         rospy.loginfo(f"Received valid index: {msg.data}")
#         current_text_index = msg.data

def next_value_callback(msg):
    global simple_response_list
    simple_response_list.append(msg.data)
    rospy.loginfo(f"Appended {msg.data} to simple_response_list: {simple_response_list}")

    # You can add debugging info if needed to check the received values
    rospy.loginfo(f"Received index: {msg.data}")

def play_audio(file_path):
    global current_process
    current_process = subprocess.Popen(["mpg123", file_path])

def stop_audio():
    global current_process
    if current_process:
        current_process.terminate()
        current_process = None

def start_gui():
    try:
        start_gui_service(EmptyRequest())
        rospy.loginfo("Start GUI service called successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call start_gui service: {e}")

def stop_gui():    
    try:
        stop_gui_service(EmptyRequest())
        rospy.loginfo("Stop GUI service called successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call stop_gui service: {e}")

def introduction():
    global introduction_file
    rospy.loginfo("Playing introduction.")
    play_audio(introduction_file)

def play_next_audio_clip():
    global current_audio_index, all_questions_exhausted, current_delay
    # if all_questions_exhausted:
    #     rospy.loginfo("No more clips to play.")
    #     return

    rospy.loginfo("Playing clip %d...", current_audio_index)
    rospy.loginfo(f"Audio Count {current_audio_index} vs Text Count {current_text_index}")
    folder_path = os.path.expanduser(simple_audio_path)

    if current_audio_index < len(simple_audio_list):
        file_name = simple_audio_list[current_audio_index]
        file_path = os.path.join(folder_path, file_name)
        if os.path.isfile(file_path):
            rospy.loginfo("Playing file: %s", file_path)
            current_delay = delay_times[random.randint(0, len(delay_times) - 1)]
            #INJECT DELAY HERE BEFORE PLAYING AUDIO
            play_audio(file_path)
        else:
            rospy.logwarn("File not found: %s", file_path)

        current_audio_index += 1
        if current_audio_index >= len(simple_audio_list):
            all_questions_exhausted = True
            rospy.loginfo("All audio clips played. Setting all_questions_exhausted to True.")
            # question_publisher.publish("All Done")  # Publish "All Done" message
    else:
        rospy.loginfo("No more clips to play.")


def publish_next_question():
    global current_text_index, next_button_count, current_audio_index, all_questions_exhausted
    rospy.loginfo(f"Next Button Count: {next_button_count} vs Text Count: {current_text_index} vs Audio Count: {current_audio_index}")

    # Check if we have exhausted all questions
    if all_questions_exhausted:
        rospy.loginfo("All out of questions.")
        question_publisher.publish("All Out of Questions")
        return

    # Ensure that the next button count is used to control publishing
    if next_button_count >= current_text_index:
        # This will ensure that the index is only updated if needed
        if current_text_index < len(simple_question_list):
            question = simple_question_list[current_text_index]
            rospy.loginfo(f"Publishing question at index {current_text_index}: {question}")
            question_publisher.publish("Question: \n" + question)
            current_audio_index = current_text_index
            current_text_index += 1

            # If this is the last question, mark all questions as exhausted
            if current_text_index >= len(simple_question_list):
                all_questions_exhausted = True
                rospy.loginfo("All questions published. Setting all_questions_exhausted to True.")
                # question_publisher.publish("All Done")
        else:
            rospy.loginfo("No more questions to publish.")


def joy_callback(data):
    global introduction_played, gui_started
    if data.buttons == (0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0):  # Start button for introduction or next question
        rospy.loginfo("Start button pressed.")

        # Play the introduction if it hasn't been played yet
        if not introduction_played and not gui_started:
            task_queue.put(introduction)
            introduction_played = True
        # Start the GUI if the introduction has been played
        elif not gui_started and introduction_played:
            start_gui()
            gui_started = True
            task_queue.put(publish_next_question)
        # Publish the next question if the GUI has been started
        # This shouldnt be necessary since the /next service is being used to trigger the next question
        else:
            task_queue.put(publish_next_question) 

    elif data.buttons == (0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0):  # Select button for stopping audio
        rospy.loginfo("Select button pressed.")
        task_queue.put(stop_audio)
        
    elif data.buttons == (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0):  # A button for next audio clip
        rospy.loginfo("A button pressed.")
        task_queue.put(play_next_audio_clip) 

def listener():
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.Subscriber("/next_value", Int32, next_value_callback)  
    rospy.spin()

def process_tasks():
    try:
        while not task_queue.empty():
            task = task_queue.get_nowait()
            task()
    except queue.Empty:
        pass
    root.after(100, process_tasks)

def signal_handler(sig, frame):
    rospy.signal_shutdown("Shutdown signal received.")
    root.quit()  # Stop the Tkinter main loop

def next_service_callback(req):
    global next_button_count
    next_button_count += 1
    publish_next_question()
    rospy.loginfo(f"Received /next service call. Count: {next_button_count}")
    return []

if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('q_and_a', anonymous=True)
    rospy.loginfo("Q&A node started.")
    init()

    # Initialize the question publisher
    question_publisher = rospy.Publisher('/questions', String, queue_size=10)

    # Initialize the service clients
    start_gui_service, stop_gui_service = init_service_clients()

    root = tk.Tk()
    root.withdraw()

    root.after(100, process_tasks)

    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    rospy.Service('/next', Empty, next_service_callback)

    signal.signal(signal.SIGINT, signal_handler)  # Catch the shutdown signal

    root.mainloop()
    listener_thread.join()  # Ensure the listener thread completes before exiting
