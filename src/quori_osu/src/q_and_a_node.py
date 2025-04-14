#!/usr/bin/env python3
'''Main node for the Q&A system. This node is responsible for playing audio clips, logging responses, and handling joystick input.'''

import csv
import json
from datetime import datetime
import random
import rospy
import os
import roslib.packages
import subprocess
import threading
import signal
from sensor_msgs.msg import Joy
import tkinter as tk
from PIL import Image, ImageTk
import queue
from std_srvs.srv import Empty, EmptyRequest
from quori_osu.srv import GetQuestion, GetQuestionRequest, KeyID, KeyIDResponse

# Lists for questions and audio files
# Pink Floyd - Dark Side of the Moon Testing
# simple_question_list = ["Money", "Breathe", "Time", "Eclipse"]
# simple_audio_list = ["Money.mp3", "Breathe.mp3", "Time.mp3", "Eclipse.mp3"]
# simple_audio_path = "/home/quori6/Music/darksideofthemoon/"
# introduction_file = os.path.join(simple_audio_path, "Brain Damage.mp3")

# Get the package's base directory (e.g., /opt/quori/src/quori_osu)
package_base_path = roslib.packages.get_pkg_dir('quori_osu')

# Simple Questions from Survey
current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
home_dir = os.path.expanduser("~")
quori_supplimental_path = os.path.join('quori_osu_supplemental')
audio_root_path = os.path.join(package_base_path,'src', quori_supplimental_path, 'q_and_a_audiofiles')
# introduction_file = os.path.join(quori_supplimental_path, 'q_an_a_audiofiles/', "SimpleIntro.mp3")

# Placeholders for KeyID info
id_string = "default_id"  # Default value
key_id_string = "7"  # Default key_id value
scale_type = "Likert"  # Default scale type

# Question masterlist filename (should be in the format *.json)
masterlist_name = 'masterlist.json'

# Create the relative paths to your key and masterlist files
logging_location = os.path.join(package_base_path, 'logs')
questions_location = os.path.join(package_base_path, 'src', quori_supplimental_path, 'questions')

# Now you can build other paths similarly
key_file_path = os.path.join(questions_location, f'key_{key_id_string}.json')
csv_file_path = os.path.join(logging_location, f'{id_string}_key{key_id_string}_log_{current_time}.csv')
masterlist_file_path = os.path.join(questions_location, masterlist_name)

# Global variables (sorry needed for threading) for questions and answers

# complexity_list = []
response_list = []
all_questions = {}

# Delay times in seconds
delay_times = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]
# Indexes and counters
current_text_index = 0
current_audio_index = 0 # this lags by one behind the text index

next_button_count = 0
current_delay = 0
rating = -1
current_process = None
task_queue = queue.Queue()
introduction_played = False
gui_started = False
updated_id = False
all_questions_exhausted = False
audio_playing = False
on_default_face = True

def init():
    """Initialize the audio output and set the delay for the first question."""
    global current_delay

    # Set the audio output to "Headphones - Built In Audio"
    set_audio_output('alsa_output.pci-0000_00_1f.3.analog-stereo')

    # Set the delay for the first question
    current_delay = random.choice(delay_times)


def init_service_clients():
    """Initialize the service clients for starting and stopping the GUI and switching faces."""
    rospy.wait_for_service('start_gui')
    rospy.wait_for_service('stop_gui')
    start_gui_service = rospy.ServiceProxy('start_gui', Empty)
    stop_gui_service = rospy.ServiceProxy('stop_gui', Empty)
    # create dictionary of face services
    faces = ['default_face', 'thinking_face', 'talking_face']
    [rospy.wait_for_service(face) for face in faces]
    face_service_dict = {face: rospy.ServiceProxy('/'+face, Empty) for face in faces}
    return start_gui_service, stop_gui_service, face_service_dict


def set_audio_output(sink_name):
    """Set the default audio output sink using pactl."""
    try:
        subprocess.run(['pactl', 'set-default-sink', sink_name], check=True)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to set audio output: {e}")


def load_json_file(file_path, default_file_path=None):
    """Load JSON data from a file, falling back to a default file if the primary file is not found."""
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


def filter_questions(master_data, key_data):
    """Filter questions from the master data based on the key data."""
    filtered_questions = []
    key_questions = key_data['questions']  # Get the list of question IDs from the key file

    for key_id in key_questions:
        for question in master_data['questions']:
            if question['id'] == key_id:  # Match the question ID with the key ID
                filtered_questions.append(question)
                break  # Stop searching after finding the match

    return filtered_questions


def initialize_questions_and_answers():
    """Initialize the questions and answers based on the key data."""

    global all_questions

    # Load the master list and key data
    master_data = load_json_file(masterlist_file_path)
    key_data = load_json_file(key_file_path)

    rospy.loginfo(f"Key Data: {key_data}")

    # Filter the questions based on the key data
    all_questions = filter_questions(master_data, key_data)

    # Log the filtered questions for debugging
    rospy.loginfo(f"Filtered Questions: {all_questions}")


    rospy.loginfo(f"Total questions: {len(all_questions)}")
    # rospy.loginfo(f"Simple Questions: {len(simple_question_list)}, Complex Questions: {len(complex_question_list)}")


# Functions


def update_csv_file_path():
    """Update the CSV file path based on the user ID and key ID."""
    global csv_file_path
    date_time = '_'.join('-'.join(current_time.split(':')).split())
    csv_file_path = os.path.join(logging_location, f"{id_string}_key{key_id_string}_log_{date_time}.csv")
    rospy.loginfo(f"Updated CSV file path to: {csv_file_path}")


def write_to_file():
    """Append the original question ID, current question, answer, delay, rating, and timestamp to a CSV file."""
    # global current_complex_writing_index, current_simple_writing_index, current_audio_index, rating
    global all_questions
    rospy.loginfo("Writing data to CSV file. Current Text Index: %d vs Response List %d", current_text_index, len(response_list))

    log_index = len(response_list) - 1 
    original_question_id = all_questions[log_index]['id']  # Ensure correct indexing
    rating_index = response_list[-1] if response_list else None

    if all_questions[log_index]['type'] == 'demo':
        rospy.loginfo("Demo Question Skipping Logging")
        return

    
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
            rospy.loginfo(f"Likert Writing")
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
        current_question = all_questions[log_index]['question']
        current_answer = all_questions[log_index]['answer']
        current_complexity = all_questions[log_index]['type']
        current_audio_file = all_questions[log_index]['audio_file']
        writer.writerow([original_question_id, current_question, current_answer, current_complexity, current_delay, rating, rating_index, scale_type, current_time, current_audio_file, masterlist_name])
    
    rospy.loginfo(f"Data logged: Question ID: {original_question_id}, Question: {current_question}, Answer: {current_answer}, Complexity: {current_complexity}, "
                  f"Delay: {current_delay}, Rating: {rating}, Rating Index: {rating_index}, Scale Type: {scale_type} Time: {current_time}, File: {current_audio_file}, Masterlist: {masterlist_name}")
    

    # Check if we have exhausted all questions
    # It needs to be here because its the last thing that happens in the order of functions
    if all_questions_exhausted:
        rospy.loginfo("All out of questions nothing written to file.")   


def handle_key_service(req):
    """Handle the KeyID service request."""
    global key_id_string, key_file_path, key_data, id_string, updated_id, scale_type, all_questions
    rospy.loginfo(f"Received User ID: {req.user_id}, Key ID: {req.key_id}, Scale Type: {req.scale_type}")

    key_id_string = req.key_id
    id_string = req.user_id
    scale_type = req.scale_type  # If needed, handle `scale_type` for further logic
    
    new_key_file_path = os.path.join(questions_location, f'key_{key_id_string}.json')
    
    # Load the new key file
    try:
        key_data = load_json_file(new_key_file_path, key_file_path)
        updated_id = True
        key_file_path = new_key_file_path
        initialize_questions_and_answers()
        
        # # Log initialized data
        rospy.loginfo(f"Questions Initialized: {[question['question'] for question in all_questions]}")
        
        # Update CSV file path to include the key_id
        update_csv_file_path()

        return KeyIDResponse(success=True)
    except Exception as e:
        rospy.logerr(f"Error processing key file: {str(e)}")
        return KeyIDResponse(success=False) 


def swap_faces(face_service):
    """Swap faces using the face service."""
    try:
        face_service(EmptyRequest())
        rospy.loginfo(f"Face Swapped Successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call face swap service: {e}")


def play_audio(file_path):
    """Original audio playing function."""
    global current_process
    if current_process is None or current_process.poll() is not None:
        current_process = subprocess.Popen(["mpg123", file_path])
        while current_process is not None and current_process.poll() is None:
            pass


def stop_audio():
    """Original stop audio function."""
    global current_process
    if current_process:
        current_process.terminate()
        current_process = None


def start_gui():
    """Sends a service call to start the GUI Node."""
    try:
        start_gui_service(EmptyRequest())
        rospy.loginfo("Start GUI service called successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call start_gui service: {e}")


def stop_gui():
    """Sends a service call to stop the GUI Node."""
    try:
        stop_gui_service(EmptyRequest())
        rospy.loginfo("Stop GUI service called successfully.")
    except rospy.ServiceException as e:
        rospy.loginfo("GUI is already stopped.")


def introduction():
    """Function to play the introduction audio. Currently unused."""
    global introduction_file
    rospy.loginfo("Playing introduction.")
    play_audio(introduction_file)


def play_with_delay(file_path, delay):
    """Function to play audio after a delay using threading."""
    def delayed_play():
        global on_default_face
        if not on_default_face:
            return
        on_default_face = False
        if delay > 0:
            swap_faces(face_service_dict['thinking_face'])
        rospy.loginfo(f"Waiting for {delay} seconds before playing.")
        rospy.sleep(delay)  # Sleep without blocking the entire program

        rospy.loginfo(f"Now playing: {file_path}")
        swap_faces(face_service_dict['talking_face'])
        play_audio(file_path)
        swap_faces(face_service_dict['default_face'])
        on_default_face = True

    threading.Thread(target=delayed_play).start()


def play_next_audio_clip():
    """Function to play the next audio clip based on the current audio index."""
    global current_audio_index, all_questions_exhausted, current_delay, all_questions

    if not all_questions_exhausted:  
        
        folder_path = os.path.expanduser(os.path.join(audio_root_path, all_questions[current_audio_index]['type']))
        file_name = all_questions[current_audio_index]['audio_file']

        file_path = os.path.join(folder_path, file_name)
        rospy.loginfo(f"Playing audio clip: {file_path}")
        play_with_delay(file_path, current_delay)

    else:
        rospy.loginfo("All questions have been exhausted.")


def handle_question_request(req):
    """Handle the question request from the service. Returns the next question as long as the question list hasnt been exhausted."""
    global current_text_index, next_button_count, current_audio_index, all_questions_exhausted, response_list, current_delay, rating, all_questions
    
    total_questions = len(all_questions)
    rospy.loginfo(
        f"Question Requested.\n Next Button Count: {next_button_count} vs Text Count: {current_text_index} vs Audio Count: {current_audio_index} vs Total Questions: {total_questions} vs Response List: {len(response_list)}")
    rospy.loginfo(f"Length of List: {len(all_questions)}")

    # Check if we have exhausted all questions
    if all_questions_exhausted:
        rospy.loginfo("All out of questions.")
        return "All Out of Questions"

    response = req.rating
    rating = response

    # If the response is not -1, it means we received a rating
    # -1 is used as a placeholder for the first question
    if response != -1:
        response_list.append(response)

        # You can add debugging info if needed to check the received values
        rospy.loginfo(f"Received index: {response}")

        # Check if we have exhausted all questions
        if len(response_list) <= len(all_questions):
            write_to_file()
        else:
            rospy.loginfo("All questions have been answered. No more logging.")
    else:
        rospy.loginfo("First question requested received.")

    # Update the current delay for the next question
    current_delay = delay_times[random.randint(0, len(delay_times) - 1)]
    
    # Check if we have exhausted all questions
    if current_text_index < total_questions and len(response_list) < total_questions:

        question = all_questions[current_text_index]['question']

        current_audio_index = current_text_index
        current_text_index += 1

        # If this is the last question, mark all questions as exhausted
        # This is redundant code and should not trigger
        if current_text_index > total_questions:
            all_questions_exhausted = True
            rospy.loginfo("All questions sent. Setting all_questions_exhausted to True.")
    else:

        all_questions_exhausted = True
        rospy.loginfo("All Out of Questions")
        question = "All Out of Questions"

    return question


def joy_callback(data):
    """Callback function for joystick input."""
    global introduction_played, gui_started

    # Start button for remote question updating
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
            #This was left in incase a remote start of the gui is desired
            start_gui()
            gui_started = True
        else:
            try:
                rospy.wait_for_service('/remote_update')  # Ensure the service is available
                remote_update_service = rospy.ServiceProxy('/remote_update', GetQuestion)

                # Create the request with the appropriate fields
                req = GetQuestionRequest()
                req.rating = -2  # Set the rating to -2 to indicate a remote update

                # Call the service
                remote_update_service(req)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

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

        
def listener():
    """Listener function for joystick input. Its broken up this way so this can run in a separate thread."""
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()


def process_tasks():
    """"Process tasks in the task queue. Used for threading with tkinter."""
    try:
        while not task_queue.empty():
            task = task_queue.get_nowait()
            task()
    except queue.Empty:
        pass
    root.after(100, process_tasks)


def signal_handler(sig, frame):
    """Handle the shutdown signal."""
    rospy.signal_shutdown("Shutdown signal received.")
    root.quit()  # Stop the Tkinter main loop


if __name__ == '__main__':
    """Main function to initialize the ROS node and start the GUI."""
    
    # Initialize the ROS node
    rospy.init_node('q_and_a', anonymous=True)
    rospy.loginfo("Q&A node started.")
    init()

    # Initialize the service clients
    start_gui_service, stop_gui_service, face_service_dict = init_service_clients()

    root = tk.Tk()
    root.withdraw()

    root.after(100, process_tasks)

    # Seperate thread for listening to the joy controller commands 
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    rospy.Service('/get_question', GetQuestion, handle_question_request)
    rospy.Service('/key_id', KeyID, handle_key_service)

    signal.signal(signal.SIGINT, signal_handler)  # Catch the shutdown signal

    root.mainloop()
    listener_thread.join()  # Ensure the listener thread completes before exiting
    rospy.loginfo("Q&A node stopped.")