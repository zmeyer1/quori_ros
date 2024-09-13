#!/usr/bin/env python3

import csv
import json
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
from quori_osu.msg import UserKey

# Lists for questions and audio files
# Pink Floyd - Dark Side of the Moon Testing
# simple_question_list = ["Money", "Breathe", "Time", "Eclipse"]
# simple_audio_list = ["Money.mp3", "Breathe.mp3", "Time.mp3", "Eclipse.mp3"]
# simple_audio_path = "/home/quori6/Music/darksideofthemoon/"
# introduction_file = os.path.join(simple_audio_path, "Brain Damage.mp3")


# Simple Questions from Survey
# simple_audio_path = "/home/quori6/Music/simple_questions/"
current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
home_dir = os.path.expanduser("~")
simple_audio_path = os.path.join(home_dir, 'Music/simple_questions/')
complex_audio_path = os.path.join(home_dir, 'Music/complex_questions/')
introduction_file = os.path.join(simple_audio_path, "SimpleIntro.mp3")
id_string = "default_id"  # Default value
key_id_string = "3"  # Default key_id value
key_number = 3 # Default key number
key_file_path = os.path.join(home_dir, f'Documents/q_and_a_json_files/key_{key_id_string}.json')
csv_file_path = os.path.join(home_dir, 'Documents/q_and_a_response_logs', f'{id_string}_key{key_id_string}_log_{current_time}.csv')
masterlist_name = 'masterlist.json'
masterlist_file_path = os.path.join(home_dir, 'Documents/q_and_a_json_files/', masterlist_name)


# Global variables for questions and answers
question_id_list = []
simple_question_list = []
simple_answer_list = []
simple_audio_list = []
complex_question_list = []
complex_answer_list = []
complex_audio_list = []
complexity_list = []
response_list = []

# Delay times in seconds
delay_times = [1, 1.5, 2, 2.5, 3]

# Indexes and counters
current_text_index = 0
current_audio_index = 0
current_simple_writing_index = 0
current_complex_writing_index = 0
current_simple_pub_index = 0
current_complex_pub_index = 0
next_button_count = 0
current_delay = 0
total_questions = 0
scale_type = "Default"
current_process = None
task_queue = queue.Queue()
introduction_played = False
gui_started = False
updated_id = False
all_questions_exhausted = False
audio_playing = False

# Initialize the audio output and delay
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


# Set the audio output
def set_audio_output(sink_name):
    try:
        subprocess.run(['pactl', 'set-default-sink', sink_name], check=True)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to set audio output: {e}")


# Load JSON data from a file
def load_json_file(file_path, default_file_path=None):
    try:
        with open(file_path, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        rospy.logwarn(f"File '{file_path}' doesn't exist, opening the default file.")
        if default_file_path:
            with open(default_file_path, 'r') as default_file:
                return json.load(default_file)
        else:
            raise FileNotFoundError(f"Neither '{file_path}' nor a default file is available.")


# Filter questions based on the key file and maintain the order
def filter_questions(master_data, key_data):
    filtered_questions = []
    key_questions = key_data['questions']  # Get the list of question IDs from the key file

    for key_id in key_questions:
        for question in master_data['questions']:
            if question['id'] == key_id:  # Match the question ID with the key ID
                filtered_questions.append(question)
                break  # Stop searching after finding the match

    return filtered_questions


# Initialize the questions and answers based on the key data
def initialize_questions_and_answers():
    global simple_question_list, simple_answer_list, simple_audio_list
    global complex_question_list, complex_answer_list, complex_audio_list
    global question_id_list, complexity_list, total_questions

    # Load the master list and key data
    master_data = load_json_file(masterlist_file_path)
    key_data = load_json_file(key_file_path)

    rospy.loginfo(f"Key Data: {key_data}")

    # Filter the questions based on the key data
    filtered_questions = filter_questions(master_data, key_data)

    # Log the filtered questions for debugging
    rospy.loginfo(f"Filtered Questions: {filtered_questions}")

    # Initialize the lists
    simple_question_list = [q['question'] for q in filtered_questions if q['type'] == 'simple']
    simple_answer_list = [q['answer'] for q in filtered_questions if q['type'] == 'simple']
    simple_audio_list = [q['audio_file'] for q in filtered_questions if q['type'] == 'simple']

    complex_question_list = [q['question'] for q in filtered_questions if q['type'] == 'complex']
    complex_answer_list = [q['answer'] for q in filtered_questions if q['type'] == 'complex']
    complex_audio_list = [q['audio_file'] for q in filtered_questions if q['type'] == 'complex']

    question_id_list = [q['id'] for q in filtered_questions]

    complexity_list = [q['type'] for q in filtered_questions]

    total_questions = len(simple_question_list) + len(complex_question_list)
    rospy.loginfo(f"Total questions: {total_questions}")
    rospy.loginfo(f"Simple Questions: {len(simple_question_list)}, Complex Questions: {len(complex_question_list)}")

    # Send the first question to the GUI
    publish_next_question()

# Functions

# Monitor the console output for audio_playing status
def monitor_console_output(command):
    global audio_playing

    # Start the subprocess with the command
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    while True:
        # Read the output line by line
        output = process.stdout.readline()
        if output == '':
            break
        if output:
            # Check for specific strings in the output
            if 'Now playing' in output:
                audio_playing = True
                print("Audio playing status: True")
            elif 'Decoding of' in output:
                audio_playing = False
                print("Audio playing status: False")
    
    # Wait for the process to finish
    process.wait()


# Get the CSV file path
def update_csv_file_path():
    global csv_file_path
    csv_file_path = os.path.join(home_dir, 'Documents/q_and_a_response_logs', 
                                 f'{id_string}_key{key_id_string}_log_{current_time}.csv')
    rospy.loginfo(f"Updated CSV file path to: {csv_file_path}")
  

# Write to the CSV file
def write_to_file():
    """Append the original question ID, current question, answer, delay, rating, and timestamp to a CSV file."""
    global current_complex_writing_index, current_simple_writing_index, current_audio_index
    rospy.loginfo("Writing data to CSV file. Current Text Index: %d vs Response List %d", current_text_index, len(response_list))


    log_index = len(response_list) - 1 
    original_question_id = question_id_list[log_index]  # Ensure correct indexing
    rating_index = response_list[-1] if response_list else None

    # Determine if the question is simple or complex based on the complexity list
    if complexity_list[log_index] == 'complex':
        index = current_complex_writing_index 
        rospy.loginfo(f"Writing Index: {index}")
        current_question = complex_question_list[index]
        current_answer = complex_answer_list[index]
        current_audio_file = complex_audio_list[index]
        current_complexity = 'Complex'
        current_complex_writing_index += 1
        rospy.loginfo(f"Complex Index: {current_complex_writing_index}")
        # current_audio_index += 1
    else:
        index = current_simple_writing_index
        rospy.loginfo(f"Writing Index: {index}")
        current_question = simple_question_list[index]
        current_answer = simple_answer_list[index]
        current_audio_file = simple_audio_list[index]
        current_complexity = 'Simple'
        current_simple_writing_index += 1
        rospy.loginfo(f"Simple Index: {current_simple_writing_index}")
        # current_audio_index += 1
    
    if rating_index is not None:
        if scale_type == "Triad":
            rospy.loginfo(f"Triad Writing")
            if rating_index == 0:
                rating = "Too Slow"
            elif rating_index == 1:
                rating = "Somewhat Slow"
            elif rating_index == 2:
                rating = "Not Slow"
        else:
            rospy.loginfo(f"Likart Writing")
            if rating_index == 0:
                rating = "Strongly Disagree"
            elif rating_index == 1:
                rating = "Disagree"
            elif rating_index == 2:
                rating = "Neutral"
            elif rating_index == 3:
                rating = "Agree"
            elif rating_index == 4:
                rating = "Strongly Agree"

    # Get the current system time
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # Check if the CSV file already exists and write the header if it doesn't
    file_exists = os.path.isfile(csv_file_path)
    
    with open(csv_file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:  # If the file doesn't exist, write the header
            writer.writerow(['Question ID', 'Question', 'Answer', 'Complexity', 'Delay', 'Rating', 'Rating Index', 'Scale Type','Timestamp', 'Audio File', 'Masterlist'])
        
        # Append the actual data, including the original question ID
        writer.writerow([original_question_id, current_question, current_answer, current_complexity, current_delay, rating, rating_index, scale_type, current_time, current_audio_file, masterlist_name])
    
    rospy.loginfo(f"Data logged: Question ID: {original_question_id}, Question: {current_question}, Answer: {current_answer}, Complexity: {current_complexity}, "
                  f"Delay: {current_delay}, Rating: {rating}, Rating Index: {rating_index}, Scale Type: {scale_type} Time: {current_time}, File: {current_audio_file}, Masterlist: {masterlist_name}")
    

    # Check if we have exhausted all questions
    # It needs to be here because its the last thing that happens in the order of functions
    if all_questions_exhausted:
        rospy.loginfo("All out of questions.")
        question_publisher.publish("All Out of Questions")

# Callback function for the /next_value topic
def next_value_callback(msg):
    global response_list, current_delay, simple_question_list, complex_question_list
    response_list.append(msg.data)

    # You can add debugging info if needed to check the received values
    rospy.loginfo(f"Received index: {msg.data}")

    # Check if we have exhausted all questions
    if len(response_list) <= len(simple_question_list) + len(complex_question_list):
        write_to_file()
    else:
        rospy.loginfo("All questions have been answered. No more logging.")

    # Update the current delay for the next question
    current_delay = delay_times[random.randint(0, len(delay_times) - 1)]


# Callback function for the key_id_topic
def key_id_callback(data):
    global key_id_string, key_file_path, key_data, id_string, updated_id
    rospy.loginfo(f"Received User ID: {data.user_id}, Key ID: {data.key_id}")
    key_id_string = data.key_id
    id_string = data.user_id
    rospy.loginfo(f"Received key_id: {key_id_string}")
    new_key_file_path = os.path.join(home_dir, f'Documents/q_and_a_json_files/key_{key_id_string}.json')
    
    # Load the new key file
    key_data = load_json_file(new_key_file_path, key_file_path)
    updated_id = True
    key_file_path = new_key_file_path
    initialize_questions_and_answers()

    rospy.loginfo(f"Simple Questions Initialized: {simple_question_list}")
    rospy.loginfo(f"Complex Questions Initialized: {complex_question_list}")

    # Update CSV file path to include the key_id
    update_csv_file_path()

def scale_type_callback(data):
    global scale_type
    scale_type = data.data
    rospy.loginfo(f"Received scale type: {scale_type}")


# OG play_audio function
def play_audio(file_path):
    global current_process
    current_process = subprocess.Popen(["mpg123", file_path])


# Blocking play_audio function, TODO implement threading
# def play_audio(file_path):
#     global current_process, audio_playing

#     if audio_playing:
#         rospy.loginfo("Audio is currently playing. Skipping play request.")
#         return

#     try:
#         # Set flag to indicate audio is playing
#         audio_playing = True
#         # Start the audio playback process
#         current_process = subprocess.Popen(["mpg123", file_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#         stdout, stderr = current_process.communicate()

#         if current_process.returncode == 0:
#             rospy.loginfo("Audio playback finished successfully.")
#         else:
#             rospy.logerr(f"An error occurred: {stderr.decode()}")

#     except Exception as e:
#         rospy.logerr(f"Error occurred during audio playback: {e}")
#     finally:
#         audio_playing = False  # Reset flag when audio finishes playing


# OG stop_audio function
def stop_audio():
    global current_process
    if current_process:
        current_process.terminate()
        current_process = None


# Blocking stop_audio function, TODO implement threading
# def stop_audio():
#     global current_process, audio_playing
#     if current_process and audio_playing:
#         rospy.loginfo("Stopping audio playback.")
#         current_process.terminate()
#         current_process.wait()  # Ensure the process has terminated
#         current_process = None
#         audio_playing = False  # Reset the flag
#     else:
#         rospy.loginfo("No audio is currently playing.")


# Sends a service call to start the GUI Node
def start_gui():
    try:
        start_gui_service(EmptyRequest())
        rospy.loginfo("Start GUI service called successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call start_gui service: {e}")


# Sends a service call to stop the GUI Node but has some issues 
def stop_gui():
    try:
        stop_gui_service(EmptyRequest())
        rospy.loginfo("Stop GUI service called successfully.")
    except rospy.ServiceException as e:
        rospy.loginfo("GUI is already stopped.")


# Function to play the introduction audio
def introduction():
    global introduction_file
    rospy.loginfo("Playing introduction.")
    play_audio(introduction_file)


# Function to play audio after a delay using threading
def play_with_delay(file_path, delay):
    """Function to play audio after a delay using threading."""
    def delayed_play():
        rospy.loginfo(f"Waiting for {delay} seconds before playing.")
        rospy.sleep(delay)  # Sleep without blocking the entire program
        rospy.loginfo(f"Now playing: {file_path}")
        play_audio(file_path)

    threading.Thread(target=delayed_play).start()

# Function to play the next audio clip

def play_next_audio_clip():
    global current_audio_index, all_questions_exhausted, current_delay

    rospy.loginfo("Playing clip %d...", current_audio_index)

    # Ensure the audio index is within the correct range
    # if current_audio_index < total_questions:
    if not all_questions_exhausted:  
        if complexity_list[current_audio_index] == 'complex':
            folder_path = os.path.expanduser(complex_audio_path)
            file_name = complex_audio_list[current_complex_writing_index]
        else:
            folder_path = os.path.expanduser(simple_audio_path)
            file_name = simple_audio_list[current_simple_writing_index]

        file_path = os.path.join(folder_path, file_name)
        play_with_delay(file_path, current_delay)

        # Update the index after playing the audio
        # current_audio_index += 1
    else:
        rospy.loginfo("All questions have been exhausted.")
        all_questions_exhausted = True


# Function to publish the next question to the GUI
def publish_next_question():
    global current_text_index, next_button_count, current_audio_index, all_questions_exhausted, current_complex_pub_index, current_simple_pub_index
    rospy.loginfo(
        f"Publish Next Question called \nNext Button Count: {next_button_count} vs Text Count: {current_text_index} vs Audio Count: {current_audio_index} vs Total Questions: {total_questions} vs Response List: {len(response_list)}")
    rospy.loginfo(f"Length of List: Simple: {len(simple_question_list)} and Complex: {len(complex_question_list)}")

        # # If this is the last question, mark all questions as exhausted
        # if current_text_index >= total_questions:
        #     all_questions_exhausted = True
        #     rospy.loginfo("All questions published. Setting all_questions_exhausted to True.")

    # Check if we have exhausted all questions
    if all_questions_exhausted:
        rospy.loginfo("All out of questions.")
        question_publisher.publish("All Out of Questions")
        return

    # Ensure that the next button count is used to control publishing
    if next_button_count + 1 >= current_text_index:
        # This will ensure that the index is only updated if needed
        if current_text_index <= total_questions and len(response_list) < total_questions:
            if complexity_list[current_text_index] == 'complex':
                # index = current_text_index - len(simple_question_list) - (len(complex_question_list) - len(simple_question_list))
                index = current_complex_pub_index
                question = complex_question_list[index]
                current_complex_pub_index += 1
            else:
                # index = current_text_index - len(complex_question_list) - (len(simple_question_list) - len(complex_question_list))
                index = current_simple_pub_index
                question = simple_question_list[index]
                current_simple_pub_index += 1
            rospy.loginfo(f"Publishing question at index {index}: {question}")
            question_publisher.publish("Question: \n Hey Quori, " + question)
            current_audio_index = current_text_index
            current_text_index += 1

            # If this is the last question, mark all questions as exhausted
            if current_text_index > total_questions:
                all_questions_exhausted = True
                rospy.loginfo("All questions published. Setting all_questions_exhausted to True.")
        else:
            all_questions_exhausted = True
            rospy.loginfo("No more questions to publish.")


# Callback function for the controller
def joy_callback(data):
    global introduction_played, gui_started

    # Start button for introduction or next question
    if data.buttons == (0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0):  
        rospy.loginfo("Start button pressed.")

        # Uncomment if you want the robot to play the introduction audio before starting the GUI
        # # Play the introduction if it hasn't been played yet
        # if not introduction_played and not gui_started:
        #     task_queue.put(introduction)
        #     introduction_played = True
        # # Start the GUI if the introduction has been played
        # elif not gui_started and introduction_played:

        if not gui_started:
            start_gui()
            gui_started = True
        else:
            rospy.loginfo("Doing Nothing.")
            # task_queue.put(publish_next_question)

    # Select button for stopping audio
    elif data.buttons == (0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0):  
        rospy.loginfo("Select button pressed.")
        task_queue.put(stop_audio)

    # A button for next audio clip
    elif data.buttons == (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0):  
        rospy.loginfo("A button pressed.")
        if updated_id == True:
            task_queue.put(play_next_audio_clip)

    elif data.buttons == (0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0):  
        rospy.loginfo("Start and Select buttons pressed.")
        task_queue.put(stop_gui)
        signal_handler(None, None)  # Call the signal handler to shut down
        


# Setup the listeners 
def listener():
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.Subscriber("/next_value", Int32, next_value_callback)
    # rospy.Subscriber('/id_topic', String, id_callback)
    rospy.Subscriber('/key_id_topic', UserKey, key_id_callback)
    rospy.Subscriber('/scale_type', String, scale_type_callback)
    rospy.spin()


# Process the tasks in the task queue
def process_tasks():
    try:
        while not task_queue.empty():
            task = task_queue.get_nowait()
            task()
    except queue.Empty:
        pass
    root.after(100, process_tasks)


# Signal handler to catch the shutdown signal
def signal_handler(sig, frame):
    rospy.signal_shutdown("Shutdown signal received.")
    root.quit()  # Stop the Tkinter main loop


# Callback function for the /next service
def next_service_callback(req):
    global next_button_count
    
    publish_next_question()
    next_button_count += 1
    rospy.loginfo(f"Received /next service call. Count: {next_button_count}")
    return []


# Main function
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('q_and_a', anonymous=True)
    rospy.loginfo("Q&A node started.")

    # Initialize the question publisher
    question_publisher = rospy.Publisher('/questions', String, queue_size=10, latch=True)

    init()

    # Initialize the service clients
    start_gui_service, stop_gui_service = init_service_clients()

    root = tk.Tk()
    root.withdraw()

    root.after(100, process_tasks)

    # monitor_thread = threading.Thread(target=monitor_console_output, args=(current_process,))
    # monitor_thread.start()

    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    rospy.Service('/next', Empty, next_service_callback)

    signal.signal(signal.SIGINT, signal_handler)  # Catch the shutdown signal

    root.mainloop()
    listener_thread.join()  # Ensure the listener thread completes before exiting
    rospy.loginfo("Q&A node stopped.")