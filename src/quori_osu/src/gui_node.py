#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from threading import Thread
import tkinter as tk
from std_msgs.msg import String, Int32
import subprocess
from quori_osu.msg import UserKey

lastest_question = "Waiting for message..."
scale_type = "Triad"

class GuiApp:
    def __init__(self, root, next_service, next_value_publisher, id_publisher, key_publisher, scale_publisher):
        self.root = root
        self.root.title("ROS Noetic GUI")
        self.root.geometry("1280x720")

        # Set the window to fullscreen
        self.root.attributes('-fullscreen', False)

        # Bind the ESC key to exit fullscreen
        self.root.bind("<Escape>", self.exit_fullscreen)

        # Store publishers and services
        self.next_service = next_service
        self.next_value_publisher = next_value_publisher
        self.id_publisher = id_publisher
        self.key_publisher = key_publisher 
        self.scale_publisher = scale_publisher        

        # Create the ID entry screen
        self.create_id_screen()

    def create_id_screen(self):
        """Set up the screen to enter the ID and integer value."""
        self.id_frame = tk.Frame(self.root)
        self.id_frame.pack(expand=True)

        # ID Entry
        self.id_label = tk.Label(self.id_frame, text="Enter ID:", font=("Arial", 24))
        self.id_label.pack(pady=10)
        self.id_entry = tk.Entry(self.id_frame, width=20, font=("Arial", 24))
        self.id_entry.pack(pady=10)

        # Integer Entry
        self.int_label = tk.Label(self.id_frame, text="Enter Key Integer Value:", font=("Arial", 24))
        self.int_label.pack(pady=10)
        self.int_entry = tk.Entry(self.id_frame, width=20, font=("Arial", 24))
        self.int_entry.pack(pady=10)

        # Toggle Button for Scale Type
        self.scale_toggle_button = tk.Button(
            self.id_frame,
            text="Scale: Triad",
            font=("Arial", 24),
            command=self.toggle_scale
        )
        self.scale_toggle_button.pack(pady=10)

        # Submit Button
        self.id_button = tk.Button(self.id_frame, text="Submit", font=("Arial", 24), command=self.publish_id)
        self.id_button.pack(pady=10)

    def toggle_scale(self):
        """Toggle between 1-3 and 1-5 scale types."""
        global scale_type
        if scale_type == "Triad":
            scale_type = "Likart"
            self.scale_toggle_button.config(text="Scale: Likart")
        else:
            scale_type = "Triad"
            self.scale_toggle_button.config(text="Scale: Triad")

        self.scale_toggle_button.pack(pady=10)

    def create_main_gui(self):
        """Set up the main GUI layout after the ID and integer are entered."""
        self.id_frame.destroy()  # Remove the ID entry widgets

        # Publish the scale type when transitioning to the main GUI
        self.scale_publisher.publish(scale_type)

        # Upper half text
        self.label = tk.Message(self.root, text="Waiting for message...", font=("Arial", 24), width=600)
        self.label.pack(pady=20)

        # Container frame for buttons
        self.container_frame = tk.Frame(self.root)
        self.container_frame.pack(expand=True)  # Center the frame vertically

        # Buttons frame
        self.frame = tk.Frame(self.container_frame)
        self.frame.pack(side=tk.TOP, pady=20)

        self.buttons = []
        self.selected_button = None  # Track the selected button

        if scale_type == "Triad":
            button_config = [
                ("Too Slow", "#FF9999", "#FFCCCC"),        # Soft red and lighter soft red
                ("Somewhat Slow", "#FFD1A6", "#FFE5CC"),   # Soft amber and lighter soft amber
                ("Not Slow", "#99FF99", "#CCFFCC"),        # Soft green and lighter soft green
            ]

            # Use the same font and size as the submit button
            button_font = ("Arial", 24)
            button_width = 20
            button_height = 2
        else:
            # Add title above the buttons when using 5-point scale
            self.title_label = tk.Label(
                self.container_frame,
                text="The Interaction with Quori was Not Slow",
                font=("Arial", 18),
                fg="black",
                pady=10
            )
            self.title_label.pack(pady=(10, 5))  # Add some padding for spacing

            button_config = [
                ("Strongly Disagree", "#FF9999", "#FFCCCC"),  # Soft red and lighter soft red
                ("Disagree", "#FFD1A6", "#FFE5CC"),           # Soft amber and lighter soft amber
                ("Neutral", "#FFFF99", "#FFFFCC"),            # Soft yellow and lighter soft yellow
                ("Agree", "#99FF99", "#CCFFCC"),              # Soft green and lighter soft green
                ("Strongly Agree", "#99FFCC", "#CCFFCC"),     # Soft green and lighter soft green
            ]

            # Use the same font and size as the submit button
            button_font = ("Arial", 24)
            button_width = 15
            button_height = 2

        for i, (label, color, selected_color) in enumerate(button_config):
            btn = tk.Button(
                self.frame,
                text=label,
                width=button_width,
                height=button_height,
                bg=color,
                activebackground=selected_color,  # Use the selected_color for active background
                font=button_font,  # Same font as the submit button
                command=lambda b=i: self.select_button(b)
            )
            btn.pack(side=tk.LEFT, anchor=tk.CENTER, padx=5)
            self.buttons.append(btn)

        # self.next_service(EmptyRequest())

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

    def publish_id(self):
        """Publish the ID and integer value entered in the entry widgets."""

        # Remove any previous error messages
        for widget in self.id_frame.winfo_children():
            if isinstance(widget, tk.Label) and widget.cget("fg") == "red":
                widget.destroy()  # Remove previous error messages

        id_string = self.id_entry.get()
        int_value = self.int_entry.get()

        try:
            int_value = int(int_value)  # Attempt to convert to integer
            rospy.loginfo(f"Attempting to publish integer value: {int_value}")

            # Check if the integer is within a valid range if needed
            if int_value < 0:  # Example check; modify as necessary
                raise ValueError("Integer value must be non-negative.")

            # Publish the key_id (integer value) separately without appending it to response_list
            if id_string:
                user_key_msg = UserKey()
                user_key_msg.user_id = id_string
                user_key_msg.key_id = int_value

                self.key_publisher.publish(user_key_msg)
                rospy.loginfo(f"Published User ID: {id_string}, Key ID: {int_value}")

            # Only move to the main GUI if everything is valid
            self.create_main_gui()
        except ValueError as e:
            rospy.logerr(f"Invalid integer value entered: {e}. No value published.")

            # Display an error message to the user
            error_label = tk.Label(self.id_frame, text="Please enter a valid integer.", fg="red", font=("Arial", 16))
            error_label.pack(pady=10)
            return

    def select_button(self, button_index):
        """Select a button, trigger the 'Next' functionality, and reset buttons."""
        global lastest_question
        # Reset the previously selected button
        if self.selected_button is not None:
            self.buttons[self.selected_button].config(relief=tk.RAISED)

        # Highlight the selected button
        self.selected_button = button_index
        self.buttons[button_index].config(relief=tk.SUNKEN)

        # Execute the functionality previously in on_next
        if lastest_question != "All Out of Questions":
            rospy.loginfo("Button pressed in GUI")
            try:
                # Publish the selected button index as an integer
                self.next_value_publisher.publish(self.selected_button)

                self.next_service(EmptyRequest())
                rospy.loginfo("Next service called successfully.")
                rospy.loginfo(f"Published selected button index: {self.selected_button}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call next service: {e}")

        # Reset the selected button and all buttons' appearance
        self.buttons[self.selected_button].config(relief=tk.RAISED)
        self.selected_button = None

    def update_label(self, text):
        """Update the label with new text."""
        if hasattr(self, 'label'):
            self.label.config(text=text)

    def run(self):
        """Run the Tkinter main loop."""
        self.root.mainloop()
        
    def close(self):
        """Close the application window safely."""
        rospy.signal_shutdown("Application window closed.")
        self.root.quit()  # Stop the main loop if it's running
        self.root.destroy()

class GuiNode:
    def __init__(self):
        rospy.init_node('gui_node')
        self.gui_app = None
        self.gui_thread = None

        # Subscriber to the /questions topic
        self.question_sub = rospy.Subscriber('/questions', String, self.question_callback)
        self.latest_question = "Waiting for message..."

        # Services to start and stop GUI
        self.start_service = rospy.Service('start_gui', Empty, self.start_gui)
        self.stop_service = rospy.Service('stop_gui', Empty, self.stop_gui)

        # Initialize the service client for /next
        self.next_service = rospy.ServiceProxy('/next', Empty)

        # Initialize the publisher for /next_value
        self.next_value_publisher = rospy.Publisher('/next_value', Int32, queue_size=10)

        # Initialize the publisher for /key_value
        # self.key_value_publisher = rospy.Publisher('/key_value', Int32, queue_size=10, latch=True)

        # Initialize the publisher for /id_topic
        self.id_publisher = rospy.Publisher('/id_topic', String, queue_size=10, latch=True)

        # Initialize the publisher for /scale_type
        self.scale_publisher = rospy.Publisher('/scale_type', String, queue_size=10, latch=True)

        # Initialize the publisher for /id_topic
        self.key_publisher = rospy.Publisher('/key_id_topic', UserKey, queue_size=10, latch=True)

    def question_callback(self, msg):
        """Callback to update the GUI label with the latest question."""
        global lastest_question
        self.latest_question = msg.data
        lastest_question = self.latest_question
        if self.gui_app is not None:
            self.gui_app.update_label(self.latest_question)

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
        self.gui_app = GuiApp(root, self.next_service, self.next_value_publisher, self.id_publisher, self.key_publisher, self.scale_publisher)
        # Ensure the latest question is shown on screen immediately
        self.gui_app.update_label(self.latest_question)
        self.gui_app.run()

    def run(self):
        """Run the ROS node."""
        rospy.loginfo("GUI Node is running")
        rospy.spin()

if __name__ == '__main__':
    node = GuiNode()
    node.run()