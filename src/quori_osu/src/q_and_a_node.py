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
import numpy as np
from sensor_msgs.msg import Joy
import queue
import enum
from std_srvs.srv import Empty, EmptyRequest
from quori_osu.srv import GetQuestion, GetQuestionRequest, KeyID, KeyIDResponse


def load_json_file(file_path, default_file_path=None):
    """Helper Function to Load JSON data from a file, falling back to a default file if the primary file is not found."""
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

class Buttons(enum.Enum):
    """Enum for button flags to be used with joystick input."""
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    SELECT = 6
    START = 7
    HOME = 8
    LEFT_STICK = 9
    RIGHT_STICK = 10

def eval_button_press(buttons, flag_idx_list, and_flags=True, exact=True):
    """evaluate a joystick button press against a list of flags. Returns True if the button press matches the flags."""
    if exact and len(flag_idx_list) != sum(buttons):
        return False # needs the same number of buttons pressed as the flags
    flag_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    for idx in flag_idx_list:
        if type(idx) is Buttons:
            idx = idx.value # Convert enum to int if necessary
        elif type(idx) is not int:
            raise TypeError(f"Expected int or enum.Enum, got {type(idx)}")
        flag_buttons[idx] = 1
    flag_buttons = tuple(flag_buttons)
    match = np.logical_and(buttons, flag_buttons)
    if and_flags:
        return sum(match) == len(flag_idx_list)
    else:
        return np.any(match)

class QandANode:
    """Main Ros Node for the Q&A"""

    delay_times = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]

    # Pathing and file management
    package_base_path = roslib.packages.get_pkg_dir('quori_osu')
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    home_dir = os.path.expanduser("~")
    quori_supplimental_path = os.path.join('quori_osu_supplemental')
    audio_root_path = os.path.join(package_base_path,'src', quori_supplimental_path, 'q_and_a_audiofiles')
    questions_location = os.path.join(package_base_path, 'src', quori_supplimental_path, 'questions')
    masterlist_name = 'masterlist.json' # must be a .json
    masterlist_file_path = os.path.join(questions_location, masterlist_name)
    logging_location = os.path.join(package_base_path, 'logs') # logging directory

    # key_file_path = os.path.join(questions_location, f'key_{key_id_string}.json') # Dont think we need these
    # csv_file_path = os.path.join(logging_location, f'{id_string}_key{key_id_string}_log_{current_time}.csv')

    # Placeholders for KeyID info
    id_string = "default_id"  # Default value
    key_id_string = "7"  # Default key_id value
    scale_type = "Likert"  # Default scale type
    
    # holds responses to the ratings
    response_list = []


    triad_naming_dict = {
        -2: "Remote Override",
        -1: "No Response",
        0: "Too Slow",
        1: "Somewhat Slow",
        2: "Not Slow"
    }
    likert_naming_dict = {
        -2: "Remote Override",
        -1: "No Response",
        0: "Totally Disagree",
        1: "Strongly Disagree",
        2: "Disagree",
        3: "Neutral",
        4: "Agree",
        5: "Strongly Agree",
        6: "Totally Agree"
    }

    # Indexes and counters
    current_text_index = 0
    current_audio_index = 0 # this lags by one behind the text index; can we change this?

    next_button_count = 0
    current_delay = 0
    rating = -1
    current_process = None
    task_queue = queue.Queue()
    gui_started = False
    updated_id = False
    all_questions_exhausted = False
    audio_playing = False
    on_default_face = True


    def __init__(self):
        """Initialize the audio output and set the delay for the first question."""

        rospy.init_node('q_and_a', anonymous=True)
        rospy.loginfo("Q&A node started.")


        # Set the audio output to "Headphones - Built In Audio" for Quori
        self.init_audio_output('alsa_output.pci-0000_00_1f.3.analog-stereo')

        # Set the delay for the first question
        self.current_delay = 0.5

        self.init_service_clients()
        self.gui_started = True

        rospy.loginfo("Joy listener started.")
        self.list = rospy.Subscriber("/joy", Joy, self.joy_callback)


        rospy.Service('/get_question', GetQuestion, self.handle_question_request)
        rospy.Service('/key_id', KeyID, self.handle_key_service)

        signal.signal(signal.SIGINT, self.shutdown)  # Catch the shutdown signal

        rospy.timer.Timer(rospy.Duration(nsecs=1e8), self.process_tasks)  # process tasks in the queue ever 100 ms

    
    def init_audio_output(self, sink_name):
        """Set the default audio output sink using pactl."""
        try:
            subprocess.run(['pactl', 'set-default-sink', sink_name], check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to set audio output: {e}")


    def init_service_clients(self):
        """Initialize the service clients for starting and stopping the GUI and switching faces."""
        rospy.wait_for_service('start_gui')
        rospy.wait_for_service('stop_gui')
        self.start_gui_service = rospy.ServiceProxy('start_gui', Empty)
        self.stop_gui_service = rospy.ServiceProxy('stop_gui', Empty)
        # create dictionary of face services
        faces = ['default_face', 'thinking_face', 'talking_face']
        [rospy.wait_for_service(face) for face in faces]
        self.face_service_dict = {face: rospy.ServiceProxy('/'+face, Empty) for face in faces}
        rospy.loginfo("All Service clients initialized.")


    def filter_questions(self, master_data, key_data):
        """Filter questions from the master data based on the key data."""
        filtered_questions = []
        key_questions = key_data['questions']  # Get the list of question IDs from the key file

        for key_id in key_questions:
            for question in master_data['questions']:
                if question['id'] == key_id:  # Match the question ID with the key ID
                    filtered_questions.append(question)
                    break  # Stop searching after finding the match

        return filtered_questions
    

    def generate_delays(self, questions, delay_times):
        """Generate an even distribution of delays for the questions."""
        num_questions = len(questions)
        if num_questions == 0:
            return []

        # Generate a list of delays that is at least as long as the number of questions
        full_delays = (delay_times * np.math.ceil(num_questions / len(delay_times)))[:num_questions]   
        return random.shuffle(full_delays)
    

    def initialize_questions_and_answers(self):
        """Initialize the questions and answers based on the key data. Also assigns delays"""

        # Load the master list and key data
        master_data = load_json_file(self.masterlist_file_path)
        key_data = load_json_file(self.key_file_path)
        rospy.loginfo(f"Key Data: {key_data}")

        # Filter the questions based on the key data
        self.all_questions = self.filter_questions(master_data, key_data)

        # generates en even distribution of delays for the questions
        self.all_delays = self.generate_delays(self.all_questions, self.delay_times)

        # Log the filtered questions for debugging
        rospy.loginfo(f"Filtered Questions: {self.all_questions}")

        rospy.loginfo(f"Total questions: {len(self.all_questions)}")


    def update_csv_file_path(self):
        """Update the CSV file path based on the user ID and key ID."""
        date_time = '_'.join('-'.join(self.current_time.split(':')).split())
        self.csv_file_path = os.path.join(self.logging_location, f"{self.id_string}_key{self.key_id_string}_log_{date_time}.csv")
        rospy.loginfo(f"Updated CSV file path to: {self.csv_file_path}")


    def write_to_file(self):
        """Append the original question ID, current question, answer, delay, rating, and timestamp to a CSV file."""
        # global current_complex_writing_index, current_simple_writing_index, current_audio_index, rating
        rospy.loginfo("Writing data to CSV file. Current Text Index: %d vs Response List %d", self.current_text_index, len(self.response_list))

        all_questions = self.all_questions
        scale_type = self.scale_type

        log_index = len(self.response_list) - 1 
        original_question_id = all_questions[log_index]['id']  # Ensure correct indexing
        rating_index, complexity_score = self.response_list[-1] if self.response_list else None

        if all_questions[log_index]['type'] == 'demo':
            rospy.loginfo("Demo Question Skipping Logging")
            return

        
        if rating_index is not None:
            if scale_type == "Triad":
                rospy.loginfo(f"Triad Writing")
                rating = self.triad_naming_dict[rating_index]
            else:
                rospy.loginfo(f"Likert Writing")
                rating = self.likert_naming_dict[rating_index]

        # Get the current system time
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Check if the CSV file already exists and write the header if it doesn't
        file_exists = os.path.isfile(self.csv_file_path)
        
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:  # If the file doesn't exist, write the header
                writer.writerow(['Question ID', 'Question', 'Answer', 'Complexity', 'Delay', 'Rating', 'Rating Index','Complexity Score' ,'Scale Type','Timestamp', 'Audio File', 'Masterlist'])
            
            # Append the actual data, including the original question ID
            current_question = all_questions[log_index]['question']
            current_answer = all_questions[log_index]['answer']
            current_complexity = all_questions[log_index]['type']
            current_audio_file = all_questions[log_index]['audio_file']
            writer.writerow([original_question_id, current_question, current_answer, current_complexity, self.current_delay, rating, rating_index, complexity_score, scale_type, current_time, current_audio_file, self.masterlist_name])
        
        rospy.loginfo(f"Data logged: Question ID: {original_question_id}, Question: {current_question}, Answer: {current_answer}, Complexity: {current_complexity}, "
                    f"Delay: {self.current_delay}, Rating: {rating}, Rating Index: {rating_index}, Complexity Score: {complexity_score}, Scale Type: {scale_type} Time: {current_time}, File: {current_audio_file}, Masterlist: {self.masterlist_name}")
        

        # Check if we have exhausted all questions
        # It needs to be here because its the last thing that happens in the order of functions
        if self.all_questions_exhausted:
            rospy.loginfo("All out of questions nothing written to file.")   


    def swap_faces(self, face_service):
        """Swap faces using the face service."""
        try:
            face_service(EmptyRequest())
            rospy.loginfo(f"Face Swapped Successfully")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call face swap service: {e}")


    def start_gui(self):
        """Sends a service call to start the GUI Node."""
        try:
            self.start_gui_service(EmptyRequest())
            rospy.loginfo("Start GUI service called successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call start_gui service: {e}")


    def stop_gui(self):
        """Sends a service call to stop the GUI Node."""
        try:
            self.stop_gui_service(EmptyRequest())
            rospy.loginfo("Stop GUI service called successfully.")
        except rospy.ServiceException as e:
            rospy.loginfo("GUI is already stopped.")

    
    def play_audio(self, file_path):
        """Use mpg123 to play an audiofile as long is one is not already playing."""
        current_process = self.current_process
        if current_process is None or current_process.poll() is not None:
            current_process = subprocess.Popen(["mpg123", file_path])
            while current_process is not None and current_process.poll() is None:
                pass


    def stop_audio(self):
        """Original stop audio function."""
        if self.current_process:
            self.current_process.terminate()
            self.current_process = None


    def play_with_delay(self, file_path, delay, should_think=True):
        """Function to play audio after a set delay."""
        def delayed_play():
            if not self.on_default_face:
                return
            self.on_default_face = False # bool to lock play feature
            if delay > 0 and should_think:
                self.swap_faces(self.face_service_dict['thinking_face'])
            rospy.loginfo(f"Waiting for {delay} seconds before playing.")
            rospy.sleep(delay)

            rospy.loginfo(f"Now playing: {file_path}")
            self.swap_faces(self.face_service_dict['talking_face'])
            self.play_audio(file_path)
            self.swap_faces(self.face_service_dict['default_face'])
            self.on_default_face = True

        threading.Thread(target=delayed_play).start()


    def play_next_audio_clip(self):
        """Function to play the next audio clip based on the current audio index."""

        if not self.all_questions_exhausted:  
            
            folder_path = os.path.expanduser(os.path.join(self.audio_root_path, self.all_questions[self.current_audio_index]['type']))
            file_name = self.all_questions[self.current_audio_index]['audio_file']

            file_path = os.path.join(folder_path, file_name)
            rospy.loginfo(f"Playing audio clip: {file_path}")
            self.play_with_delay(file_path, self.current_delay)

        else:
            rospy.loginfo("All questions have been exhausted.")


    def handle_key_service(self, req):
        """Handle the KeyID service request."""
        rospy.loginfo(f"Received User ID: {req.user_id}, Key ID: {req.key_id}, Scale Type: {req.scale_type}")

        self.key_id_string = req.key_id
        self.id_string = req.user_id
        self.scale_type = req.scale_type
        
        self.key_file_path = os.path.join(self.questions_location, f'key_{self.key_id_string}.json')        
        try:
            self.initialize_questions_and_answers()
            
            rospy.loginfo(f"Questions Initialized: {[question['question'] for question in self.all_questions]}")
            
            # Update CSV file path to include the key_id
            self.update_csv_file_path()

            return KeyIDResponse(success=True)
        except Exception as e:
            rospy.logerr(f"Error processing key file: {str(e)}")
            return KeyIDResponse(success=False) 


    def handle_question_request(self, req):
        """Handle the question request from the service. Returns the next question as long as the question list hasnt been exhausted."""        
        total_questions = len(self.all_questions)
        rospy.loginfo(
            f"Question Requested.\n" +
            f"\tNext Button Count: {self.next_button_count}\n" +
            f"\tText Count: {self.current_text_index}\n" +
            f"\tAudio Count: {self.current_audio_index}\n" +
            f"\tTotal Questions: {total_questions}\n" +
            f"\tResponse List: {len(self.response_list)}\n"
        )
        rospy.loginfo(f"Length of List: {total_questions}")

        # Check if we have exhausted all questions
        if self.all_questions_exhausted:
            rospy.loginfo("All out of questions.")
            return "All Out of Questions"

        self.rating = req.rating, req.complexity

        # If the response is not -1, it means we received a rating
        # -1 is used as a placeholder for the first question
        if self.rating != -1:
            self.response_list.append(self.rating)

            # You can add debugging info if needed to check the received values
            rospy.loginfo(f"Received index: {self.rating}")

            # Check if we have exhausted all questions
            if len(self.response_list) <= len(self.all_questions):
                self.write_to_file()
            else:
                rospy.loginfo("All questions have been answered. No more logging.")
        else:
            rospy.loginfo("First question requested received.")

        # Update the current delay for the next question # TODO: FIX distibution
        self.current_delay = random.choice(self.delay_times)
        
        # Check if we have exhausted all questions
        if self.current_text_index < total_questions and len(self.response_list) < total_questions:

            question = self.all_questions[self.current_text_index]['question']

            self.current_audio_index = self.current_text_index
            self.current_text_index += 1

            # If this is the last question, mark all questions as exhausted
            # This is redundant code and should not trigger
            if self.current_text_index > total_questions:
                self.all_questions_exhausted = True
                rospy.loginfo("All questions sent. Setting all_questions_exhausted to True.")
        else:
            self.all_questions_exhausted = True
            rospy.loginfo("All Out of Questions")
            question = "All Out of Questions"

        return question


    def joy_callback(self, data):
        """Callback function for joystick input."""
        # Start button for remote question updating
        if eval_button_press(data.buttons, [Buttons.START]):
            rospy.loginfo("Start button pressed.")

            if not self.gui_started:
                #This was left in incase a remote start of the gui is desired
                self.start_gui()
                self.gui_started = True
            else:
                try:
                    rospy.wait_for_service('/remote_update')  # Ensure the service is available
                    # its gross to have this declared everytime we skip a question, but its better than having the proxy next to the server
                    remote_update_service = rospy.ServiceProxy('/remote_update', GetQuestion)

                    # Create the request with the appropriate fields
                    req = GetQuestionRequest()
                    req.rating = -2  # Set the rating to -2 to indicate a remote update
                    req.complexity = -2

                    # Call the service
                    remote_update_service(req)
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")

        # Select button for stopping audio
        elif eval_button_press(data.buttons, [Buttons.SELECT]):
            rospy.loginfo("Select button pressed.")
            self.task_queue.put(self.stop_audio)

        # A button for next audio clip
        elif eval_button_press(data.buttons, [Buttons.A]): 
            rospy.loginfo("A button pressed.")
            self.task_queue.put(self.play_next_audio_clip)

        elif eval_button_press(data.buttons, [Buttons.START, Buttons.SELECT]):  
            rospy.loginfo("Start and Select buttons pressed.")
            self.task_queue.put(self.stop_gui)
            self.shutdown(None, None)  # Call the signal handler to shut down

        
    def listener(self):
        """Listener function for joystick input. Its broken up this way so this can run in a dedicated thread."""
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.spin()


    def process_tasks(self, event):
        """"Process tasks in the task queue. Used for a timer."""
        try:
            while not self.task_queue.empty():
                task = self.task_queue.get_nowait()
                task()
        except queue.Empty:
            pass


    def shutdown(self, sig, frame):
        """Handle the shutdown signal."""
        rospy.signal_shutdown("Shutdown signal received.")
        self.stop_gui()
        rospy.loginfo("Q&A node stopped.")



if __name__ == '__main__':
    """Main function to initialize the ROS node and start the GUI."""
   
    q_and_a_node = QandANode()
    rospy.spin()