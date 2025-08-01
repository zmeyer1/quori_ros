#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import sys
from threading import Thread
import tkinter as tk
from tkinter import ttk
import subprocess
import random
import time
from quori_osu.srv import GetQuestion, KeyID, KeyIDRequest

# Global variables
scale_type = "Likert" # Default scale type

# Color constants
LIGHT_BLUE = "#d4e1ff"
GREEN = "#8CD47E"
LIGHT_GREEN = "#C4E6C1"

class GuiApp:
    """Main GUI application class."""

    latest_question = "Waiting for message..." # Default message

    def __init__(self, root, question_service, key_id_service, question_label):
        """Initialize the GUI application."""
        self.root = root
        self.question_label = question_label

        # Global Flags 
        self.use_UUID = True
        self.demo_question = True 
        self.gen_question_key = True # False ads the entry box for key value 

        self.root.title("ROS Noetic GUI")
        self.root.geometry("1280x720")

        self.root.configure(bg=LIGHT_BLUE)

        # Set the window to fullscreen
        self.root.attributes('-fullscreen', True)

        # Bind the ESC key to exit fullscreen
        # self.root.bind("<Escape>", self.exit_fullscreen)
        self.root.bind("<Escape>", self.toggle_fullscreen)
        self.root.bind("<Control-c>", lambda e: self.root.quit())

        # Initialize services
        self.question_service = question_service
        self.key_id_service = key_id_service

        # Configure Radiobuttons
        s = ttk.Style()
        s.theme_use('default')

        s.layout('TRadiobutton',
                [('Radiobutton.padding',
                {'children':
                    [('Radiobutton.indicator', {'side': 'top', 'sticky': ''}), # Just need to change indicator's 'side' value
                    ('Radiobutton.focus', {'side': 'left',
                                            'children':
                                            [('Radiobutton.label', {'sticky': 'nswe'})],
                                            'sticky': ''})],
                    'sticky': 'nswe'})])
        
        s.configure('TRadiobutton', font=('Arial', 18), background=LIGHT_BLUE, foreground='black', padding=10)


        # Create the ID entry screen
        self.create_id_screen()


    def create_id_screen(self):
        """Set up the screen to enter the ID and integer value."""
        self.id_frame = tk.Frame(self.root, bg=LIGHT_BLUE)
        self.id_frame.pack(expand=True)

        self.title = tk.Label(self.id_frame, text="Welcome to the Quori Q & A!", font=("Arial", 24), bg=LIGHT_BLUE)
        self.title.pack(pady=10)

        if not self.use_UUID:
            # Name Entry
            self.id_label = tk.Label(self.id_frame, text="Enter Name:", font=("Arial", 24), bg=LIGHT_BLUE)
            self.id_label.pack(pady=10)
            self.id_entry = tk.Entry(self.id_frame, width=20, font=("Arial", 24))
            self.id_entry.pack(pady=10)
        else:
            self.UUID = str(int(round(time.time())))[4:]
            self.id_label = tk.Label(self.id_frame, text=f"Participant ID: {self.UUID}", font=("Arial", 24), bg=LIGHT_BLUE)
            self.id_label.pack(pady=10)

        if self.gen_question_key == False:
            if self.question_label is None:
                # Integer Entry
                self.int_label = tk.Label(self.id_frame, text="Enter Key Integer Value:", font=("Arial", 24), bg=LIGHT_BLUE)
                self.int_label.pack(pady=10)
                self.int_entry = tk.Entry(self.id_frame, width=20, font=("Arial", 24))
                self.int_entry.pack(pady=10)
        else:
            # Generate a random key integer between 0 and 99
            self.random_key_value = random.randint(0, 99)  # Random int between 0 and 99

        

        # Toggle Button for Scale Type
        self.scale_toggle_button = tk.Button(
            self.id_frame,
            text=f"Scale: {scale_type}",
            font=("Arial", 24),
            command=self.toggle_scale,
            bg = LIGHT_BLUE,
            activebackground = LIGHT_BLUE
        )
        self.scale_toggle_button.pack(pady=10)

        # Submit Button
        self.id_button = tk.Button(
            self.id_frame,
            text="Start",
            font=("Arial", 24), 
            command=self.send_key_id,
            bg=GREEN,
            activebackground = LIGHT_GREEN,
            )
        self.id_button.pack(pady=10)


    def toggle_scale(self):
        """Toggle between 1-3 "Triad" and 1-5 "Likert" scale types."""
        global scale_type

        if scale_type == "Triad":
            scale_type = "Likert"
            self.scale_toggle_button.config(text="Scale: Likert")
        else:
            scale_type = "Triad"
            self.scale_toggle_button.config(text="Scale: Triad")

        self.scale_toggle_button.pack(pady=10)


    def create_main_gui(self):
        """Set up the main GUI layout after the ID and integer are entered."""

        # Clear the ID entry frame
        self.id_frame.destroy()

        # Request the first question
        response = self.question_service(-1, -1)
        self.latest_question = response.question
        rospy.loginfo(f"First question received from service: {self.latest_question}")

        # Container frame for the question label and the question itself
        self.question_frame = tk.Frame(self.root, bg=LIGHT_BLUE)
        self.question_frame.pack(pady=20)

        # Add a label that says "Question:" above the actual question
        self.question_label = tk.Label(self.question_frame, text="Question:", font=("Arial", 24), fg="black", bg=LIGHT_BLUE)
        self.question_label.pack()

        # Display the actual question
        self.label = tk.Message(self.question_frame, text=self.latest_question, font=("Arial", 24), width=600, bg=LIGHT_BLUE)
        self.label.pack(pady=10)

        # Container frame for buttons and title label
        self.container_frame = tk.Frame(self.root, bg=LIGHT_BLUE)
        self.container_frame.pack(expand=True)  # Center the frame vertically
        
        if scale_type == "Triad":
            # Title label for interaction question
            self.title_label = tk.Label(
                self.container_frame,
                text="How would you rate the response time of Quori?",  # This label is now above the buttons
                font=("Arial", 22),
                fg="black",
                pady=10,
                bg=LIGHT_BLUE,
            )
        else:
            # Title label for interaction question
            self.title_label = tk.Label(
                self.container_frame,
                text="The timing of Quori's response was natural",  # This label is now above the buttons
                font=("Arial", 22),
                fg="black",
                pady=10,
                bg=LIGHT_BLUE,
            )
        self.title_label.pack(pady=(10, 5))  # Add some padding for spacing

        # Upper frame
        self.upper_frame = tk.Frame(self.container_frame, bg=LIGHT_BLUE)
        self.upper_frame.pack(side=tk.TOP, pady=0)

        width = self.upper_frame.winfo_width()
        height = self.upper_frame.winfo_height()
        
        self.buttons = []
        self.selected_button = None  # Track the selected button

        if scale_type == "Triad":
            button_config = [
                ("Too Slow  ", "#FF9999", "#FFCCCC"),        # Soft red and lighter soft red
                ("Somewhat Slow", "#FFD1A6", "#FFE5CC"),   # Soft amber and lighter soft amber
                ("Not Slow", "#99FF99", "#CCFFCC"),        # Soft green and lighter soft green
            ]

            # Use the same font and size as the submit button
            button_font = ("Arial", 20)
            button_width = width // 4
            button_height = height // 6
        else:
            button_config = [
                ("Totally Disagree", "#FF8981", "#FFCCC7"),  # Softer red and lighter soft red
                ("Strongly Disagree", "#FF8981", "#FFCCC7"),  # Softer red and lighter soft red
                ("Disagree", "#FFB54C", "#FFD6A1"),           # Same orange and lighter orange
                ("Neutral", "#F8D66D", "#FAE6A8"),            # Same yellow and lighter yellow
                ("Agree", GREEN, LIGHT_GREEN),              # Same green and lighter green
                ("Strongly Agree", "#6FAF72", "#AFCFB2"),     # Darker green and lighter green
                ("Totally Agree", "#6FAF72", "#AFCFB2"),     # Darker green and lighter green
            ]

            # Use the same font and size as the submit button
            button_font = ("Arial", 16)
            button_width = width // 6
            button_height = height // 6

        self.selected_rating = tk.IntVar(value=-1)
        self.selected_complexity = tk.IntVar(value=-1)

        for i, (label, color, selected_color) in enumerate(button_config):
            btn = ttk.Radiobutton(
                self.upper_frame,
                text=label,
                variable=self.selected_rating,
                value=i,
            )
            btn.pack(side=tk.LEFT, anchor=tk.CENTER, padx=5)
            self.buttons.append(btn)

        self.lower_frame = tk.Frame(self.container_frame, bg=LIGHT_BLUE)
        self.lower_frame.pack(side=tk.TOP, pady=50)

        # Complexity Score Question
        self.complex_label = tk.Label(
                self.lower_frame,
                text="The question you asked Quori was complicated",
                font=("Arial", 22),
                fg="black",
                pady=10,
                bg=LIGHT_BLUE,
            )
        self.complex_label.pack(anchor=tk.CENTER)  # Add some padding for spacing

        for i, (label, color, selected_color) in enumerate(button_config):
            btn = ttk.Radiobutton(
                self.lower_frame,
                text=label,
                variable=self.selected_complexity,
                value=i,
            )
            btn.pack(side=tk.LEFT, padx=5)
            self.buttons.append(btn)


        # Add a submit button to send the selected rating
        self.submit_button = tk.Button(
            self.container_frame,
            text="Submit",
            font=("Arial", 24),
            command=self.submit_rating,
            bg=GREEN,
            activebackground=LIGHT_GREEN,
        )
        self.submit_button.pack(pady=10)


        # Adjust row height and column width for responsiveness
        for col in range(5):
            self.upper_frame.grid_columnconfigure(col, weight=1)

        self.upper_frame.update_idletasks()


    def calculate_dynamic_font_size(self):
        """Calculate a dynamic font size based on available space."""
        width = self.upper_frame.winfo_width()  # Get the width of the button container frame
        available_width = width // 5  # Divide by 5 because we have 5 buttons TODO: this isn't dynamic...
        font_size = available_width // 10  # Adjust this ratio to get the ideal font size
        
        # Ensure a minimum font size to prevent being too small
        return max(font_size, 14)  # Ensure the font size is at least 14


    def bring_to_front(self):
        """Bring the window to the front and ensure it stays on top."""
        self.root.focus_force()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)  # Reset -topmost attribute

        try:
            # Get the current window ID using xprop
            window_id = subprocess.check_output(
                ["xprop", "-root", "_NET_ACTIVE_WINDOW"],
                text=True
            ).strip().split()[-1]

            # Use wmctrl to raise the window to the front
            subprocess.run(["wmctrl", "-i", "-a", window_id])
        except Exception as e:
            rospy.logwarn(f"Failed to bring window to front: {e}")


    def exit_fullscreen(self, event=None):
        """Exit fullscreen mode."""
        self.root.attributes('-fullscreen', False)


    def toggle_fullscreen(self, event=None):
        """Exit fullscreen mode."""
        self.root.attributes('-fullscreen', not self.root.attributes('-fullscreen'))


    def send_key_id(self):
        """Send the user ID, key ID, and scale type to the /key_id service."""

        # Remove any previous error messages
        for widget in self.id_frame.winfo_children():
            if isinstance(widget, tk.Label) and widget.cget("fg") == "red":
                widget.destroy()  # Remove previous error messages
        
        if not self.use_UUID:
            id_string = self.id_entry.get()
        else:
            id_string = self.UUID

        if self.gen_question_key == False:
            int_value = self.int_entry.get()
        else:
            int_value = self.random_key_value

        try:
            int_value = int(int_value)  # Attempt to convert to integer
            rospy.loginfo(f"Attempting to send key ID: {int_value}")

            # Check if the integer is within a valid range if needed
            if int_value < 0:  
                raise ValueError("Integer value must be non-negative.")

            if id_string:
                # Create the request for the KeyID service
                key_id_srv = KeyIDRequest()  # Use KeyIDRequest instead of KeyID
                key_id_srv.user_id = id_string
                key_id_srv.key_id = int_value
                key_id_srv.scale_type = scale_type

                # Call the key_id_service with the request and capture the response
                response = self.key_id_service(key_id_srv)

                # Check the response
                if response.success:
                    rospy.loginfo(f"Service call successful: User ID: {id_string}, Key ID: {int_value}, Scale Type: {scale_type}")
                    # Proceed to the main GUI if successful
                    self.create_main_gui()
                else:
                    rospy.logerr("Service call failed. Success flag is false.")
                    error_label = tk.Label(self.id_frame, text="Service call failed. Please try again.", fg="red", font=("Arial", 16))
                    error_label.pack(pady=10)

        except ValueError as e:
            rospy.logerr(f"Invalid integer value entered: {e}. No value sent.")

            # Display an error message to the user
            error_label = tk.Label(self.id_frame, text="Please enter a valid integer.", fg="red", font=("Arial", 16))
            error_label.pack(pady=10)
            return

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /key_id failed: {e}")

            # Display an error message to the user
            error_label = tk.Label(self.id_frame, text="Service call failed. Please try again.", fg="red", font=("Arial", 16))
            error_label.pack(pady=10)
            return


    def submit_rating(self):
        """Submit the selected rating and call the question service."""
        selected_rating = self.selected_rating.get()
        selected_complexity = self.selected_complexity.get()
        if selected_rating == -1 or selected_complexity == -1:
            rospy.logwarn("No rating or complexity selected. Please select a rating before submitting.")
            return
        self.submit_button.config(relief=tk.SUNKEN)
        try:
            rospy.loginfo(f"Calling question service with selected button index: {selected_rating}")
            # Call the service with the selected button index
            response = self.question_service(selected_rating, selected_complexity)  # Capture the response
            self.latest_question = response.question  # Update the latest question
            self.update_label(self.latest_question)  # Update the GUI with the new question
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call question service: {e}")
        self.submit_button.config(relief=tk.RAISED)
        self.selected_rating.set(-1)
        self.selected_complexity.set(-1)
    

    def update_label(self, text):
        """Update the label with new text."""
        if hasattr(self, 'label'):
            if text == 'All Out of Questions':
                for btn in self.buttons:
                    btn.destroy()
                self.title_label.destroy()
                self.complex_label.destroy()
                self.question_label.destroy()
                self.submit_button.destroy()
                self.thank_you_text = tk.Label(
                self.container_frame,
                text="Thank you for your Participation!",
                font=("Arial", 24),
                fg="black",
                pady=10,
                bg=LIGHT_BLUE,
                )
                self.thank_you_text.pack()

                
            self.label.config(text=text)


    def update_label_with_latest_question(self):
        """Update the label with the latest question."""
        self.update_label(self.latest_question)  # Use the global variable
        self.root.after(1000, self.update_label_with_latest_question)  # Continue updating


    def run(self):
        """Run the Tkinter main loop."""
        self.root.mainloop()


    def close(self):
        """Close the application window safely."""
        rospy.signal_shutdown("Application window closed.")
        self.root.quit()  # Stop the main loop if it's running
        self.root.destroy()


class GuiNode:
    """ROS Node for the GUI application."""

    def __init__(self, question_label = None):
        """Initialize the GUI Node."""
        rospy.init_node('gui_node')
        self.gui_app = None
        self.gui_thread = None
        self.question_label = question_label # from commandline, which set of questions to run
        self.latest_question = "Waiting for message..."

        # Services to start and stop GUI
        self.start_service = rospy.Service('start_gui', Empty, self.start_gui)
        self.stop_service = rospy.Service('stop_gui', Empty, self.stop_gui)

        # Initialize the service client for /get_question
        # rospy.wait_for_service('/get_question')
        self.get_question_service = rospy.ServiceProxy('/get_question', GetQuestion, self.request_question)

        self.remote_update = rospy.Service('/remote_update', GetQuestion, self.request_question)

        # Initialize the service client for /key_id
        rospy.wait_for_service('/key_id')
        self.key_id_service = rospy.ServiceProxy('/key_id', KeyID)


    def request_question(self, req):
        """Request a question from the service."""
        try:
            response = self.get_question_service(req)
            self.latest_question = response.question
            print(f"New Question: {self.latest_question}")
            # Update your GUI with the received question
            self.gui_app.update_label(self.latest_question)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


    def start_gui(self, req):
        """Start the GUI application."""
        if self.gui_app is None:
            rospy.loginfo("Starting GUI")
            self.gui_thread = Thread(target=self.launch_gui)
            self.gui_thread.start()
        else:
            rospy.logwarn("GUI is already running")
        return EmptyResponse()


    def stop_gui(self, req):
        """Stop the GUI application."""
        if self.gui_app is not None:
            rospy.loginfo("Stopping GUI")
            self.gui_app.close()
            rospy.loginfo("GUI closed")
            self.gui_thread.join()  # Wait for the GUI thread to finish
            rospy.loginfo("GUI thread finished")
            self.gui_app = None
        else:
            rospy.logwarn("GUI is not running")
        return EmptyResponse()


    def launch_gui(self):
        """Launch the Tkinter GUI application."""
        root = tk.Tk()
        self.gui_app = GuiApp(root, self.get_question_service, self.key_id_service, self.question_label)
        # Ensure the latest question is shown on screen immediately
        self.gui_app.update_label(self.latest_question)
        self.gui_app.run()

    def run(self):
        """Run the ROS node."""
        rospy.loginfo("GUI Node is running")
        self.launch_gui()
        rospy.spin()


if __name__ == '__main__':
    """Main entry point for the GUI Node."""

    try:
        question_label = int(sys.argv[1])
    except (IndexError, TypeError):
        rospy.loginfo("No Question label found, prompting during startup")
        question_label = None
    node = GuiNode(question_label)
    node.run()