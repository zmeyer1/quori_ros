#!/usr/bin/env python3

import csv
from datetime import datetime
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
    "what is the sum of 2+2?",
    "how many days are in a week?",
    "how many letters are in the English alphabet?",
    "how many minutes are in an hour?",
    "how many sides does a triangle have?",
    "what color is the sky",
    "what is the capital of France?",
    "what is the capital of Italy?",
    "what is the capital of the United States?",
    "what is the currency of Japan?",
    "what is the largest planet in our solar system?",
    "what is the square root of 16?"
]

simple_answer_list = [
    "The sum of two plus two is four.",
    "There are seven days in a week.",
    "There are twenty-six letters in the English alphabet.",
    "There are sixty minutes in an hour.",
    "A triangle has three sides.",
    "The sky is blue.",
    "The capital of France is Paris.",
    "The capital of Italy is Rome.",
    "The capital of the United States is Washington D.C.",
    "The currency of Japan is the yen.",
    "The largest planet in our solar system is Jupiter.",
    "The square root of sixteen is four."
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

audio_playing = False

def init():
    global current_delay
    # Set the audio output to "Headphones - Built In Audio"
    set_audio_output('alsa_output.pci-0000_00_1f.3.analog-stereo')
    # Set the delay for the first question
    current_delay = delay_times[random.randint(0, len(delay_times) - 1)]


# Initialize the service clients
def init_service_clients():
    rospy.wait_for_service('start_gui')
    rospy.wait_for_service('stop_gui')
    start_gui_service = rospy.ServiceProxy('start_gui', Empty)
    stop_gui_service = rospy.ServiceProxy('stop_gui', Empty)
    return start_gui_service, stop_gui_service

def set_audio_output(sink_name):
    try:
        subprocess.run(['pactl', 'set-default-sink', sink_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to set audio output: {e}")

# CSV file path
def get_csv_file_path():
    # current_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    return f'/home/quori6/Documents/simple_response_logs/log_{current_time}.csv'

csv_file_path = get_csv_file_path()

def write_to_file():
    """Append the current question, answer, delay, rating, and timestamp to a CSV file."""
    rospy.loginfo("Writing data to CSV file. Current Text Index: %d vs Response List %d", current_text_index, len(simple_response_list))
    # log_index = current_text_index - 2
    log_index = len(simple_response_list) - 1 
    current_question = simple_question_list[log_index]
    current_answer = simple_answer_list[log_index]
    current_audio_file = simple_audio_list[log_index]
    rating_index = simple_response_list[-1] if simple_response_list else None
    if rating_index is not None:
        if rating_index == 0:
            rating = "Too Slow"
        elif rating_index == 1:
            rating = "Somewhat Slow"
        elif rating_index == 2:
            rating = "Not Slow"

    # Get the current system time
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # Append data to the CSV file
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([current_question, current_answer, current_delay, rating, rating_index, current_time, current_audio_file])
    
    rospy.loginfo(f"Data logged: Question: {current_question}, Answer: {current_answer}, "
                  f"Delay: {current_delay}, Rating: {rating}, Rating Index: {rating_index}, Time: {current_time}, File: {current_audio_file}")

def next_value_callback(msg):
    global simple_response_list, current_delay, simple_question_list
    simple_response_list.append(msg.data)
    rospy.loginfo(f"Appended {msg.data} to simple_response_list: {simple_response_list}")

    # You can add debugging info if needed to check the received values
    rospy.loginfo(f"Received index: {msg.data}")

    rospy.loginfo(f"Len of simple response list {len(simple_response_list)} vs Simple Question List Length: {len(simple_question_list)}")
    if len(simple_response_list) <= len(simple_question_list):
        write_to_file()

    current_delay = delay_times[random.randint(0, len(delay_times) - 1)]


# def play_audio(file_path):
#     global current_process
#     current_process = subprocess.Popen(["mpg123", file_path])

def play_audio(file_path):
    global current_process, audio_playing

    if audio_playing:
        rospy.loginfo("Audio is currently playing. Skipping play request.")
        return

    try:
        # Set flag to indicate audio is playing
        audio_playing = True
        # Start the audio playback process
        current_process = subprocess.Popen(["mpg123", file_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = current_process.communicate()

        if current_process.returncode == 0:
            rospy.loginfo("Audio playback finished successfully.")
        else:
            rospy.logerr(f"An error occurred: {stderr.decode()}")

    except Exception as e:
        rospy.logerr(f"Error occurred during audio playback: {e}")
    finally:
        audio_playing = False  # Reset flag when audio finishes playing


# def stop_audio():
#     global current_process
#     if current_process:
#         current_process.terminate()
#         current_process = None

def stop_audio():
    global current_process, audio_playing
    if current_process and audio_playing:
        rospy.loginfo("Stopping audio playback.")
        current_process.terminate()
        current_process.wait()  # Ensure the process has terminated
        current_process = None
        audio_playing = False  # Reset the flag
    else:
        rospy.loginfo("No audio is currently playing.")


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


def play_with_delay(file_path, delay):
    """Function to play audio after a delay using threading."""
    def delayed_play():
        rospy.loginfo(f"Waiting for {delay} seconds before playing.")
        rospy.sleep(delay)  # Sleep without blocking the entire program
        rospy.loginfo(f"Now playing: {file_path}")
        play_audio(file_path)

    threading.Thread(target=delayed_play).start()


def play_next_audio_clip():
    global current_audio_index, all_questions_exhausted, current_delay

    rospy.loginfo("Playing clip %d...", current_audio_index)
    rospy.loginfo(f"Audio Count {current_audio_index} vs Text Count {current_text_index} vs Next Button Count {next_button_count}")
    folder_path = os.path.expanduser(simple_audio_path)

    if current_audio_index < len(simple_audio_list):
        file_name = simple_audio_list[current_audio_index]
        file_path = os.path.join(folder_path, file_name)
        if os.path.isfile(file_path):
            rospy.loginfo("Playing file: %s", file_path)

            play_with_delay(file_path, current_delay)  # Use the threaded delay function

        else:
            rospy.logwarn("File not found: %s", file_path)

        if current_audio_index < next_button_count:
            current_audio_index += 1
        
        if current_audio_index >= len(simple_audio_list):
            all_questions_exhausted = True
            rospy.loginfo("All audio clips played. Setting all_questions_exhausted to True.")
            # question_publisher.publish("All Done")  # Publish "All Done" message
    else:
        rospy.loginfo("No more clips to play.")


def publish_next_question():
    global current_text_index, next_button_count, current_audio_index, all_questions_exhausted
    rospy.loginfo(
        f"Next Button Count: {next_button_count} vs Text Count: {current_text_index} vs Audio Count: {current_audio_index}")

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
            question_publisher.publish("Question: \n Hey Quori, " + question)
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
    rospy.loginfo("Q&A node stopped.")