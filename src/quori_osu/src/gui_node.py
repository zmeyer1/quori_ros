#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from threading import Thread
import tkinter as tk
from std_msgs.msg import String, Int32
import os
import subprocess

# class GuiApp:
#     def __init__(self, root, next_service, next_value_publisher):
#         self.root = root
#         self.root.title("ROS Noetic GUI")
#         self.root.geometry("1280x720")

#         # Set the window to fullscreen
#         self.root.attributes('-fullscreen', False)
#         # self.root.attributes('-fullscreen', True)

#         # Focus the window on startup
#         self.root.after(100, self.bring_to_front)

#         # Bind the ESC key to exit fullscreen
#         self.root.bind("<Escape>", self.exit_fullscreen)

#         self.next_service = next_service  # Store the next_service proxy
#         self.next_value_publisher = next_value_publisher  # Store the publisher for the next value

#         # Upper half text
#         self.label = tk.Message(root, text="Waiting for message...", font=("Arial", 16), width=600)
#         self.label.pack(pady=20)

#         # Buttons frame
#         self.frame = tk.Frame(root)
#         self.frame.pack(side=tk.TOP, pady=20)

#         self.buttons = []
#         self.selected_button = None  # Track the selected button

#         for i in range(5):
#             btn = tk.Button(self.frame, text=f"Button {i+1}", width=10, height=2,
#                             command=lambda b=i: self.select_button(b))
#             btn.pack(side=tk.LEFT, anchor=tk.CENTER, padx=5)
#             self.buttons.append(btn)

#         # Next button
#         self.next_button = tk.Button(root, text="Next", width=10, height=2, state=tk.DISABLED, command=self.on_next)
#         self.next_button.pack(side=tk.BOTTOM, pady=10)

#     def bring_to_front(self):
#         """Bring the window to the front and ensure it stays on top."""
#         # Force focus on the window
#         self.root.focus_force()
#         self.root.attributes('-topmost', True)
#         self.root.after_idle(self.root.attributes, '-topmost', False)  # Reset -topmost attribute
        
#         # Attempt to bring the window to the front using wmctrl (Linux specific)
#         try:
#             # Get the current window ID using xprop
#             window_id = subprocess.check_output(
#                 ["xprop", "-root", "_NET_ACTIVE_WINDOW"],
#                 text=True
#             ).strip().split()[-1]

#             # Use wmctrl to raise the window to the front
#             subprocess.run(["wmctrl", "-i", "-a", window_id])
#         except Exception as e:
#             rospy.logwarn(f"Failed to bring window to front: {e}")

#     def exit_fullscreen(self, event=None):
#         """Exit fullscreen mode."""
#         self.root.attributes('-fullscreen', False)

#     def select_button(self, button_index):
#         """Select a button and enable the Next button."""
#         # Reset the previously selected button
#         if self.selected_button is not None:
#             self.buttons[self.selected_button].config(relief=tk.RAISED)

#         # Highlight the selected button
#         self.selected_button = button_index
#         self.buttons[button_index].config(relief=tk.SUNKEN)

#         # Enable the Next button
#         self.next_button.config(state=tk.NORMAL)

#     def on_next(self):
#         """Handle the Next button press."""
#         rospy.loginfo("Next button pressed in GUI")
#         try:
#             self.next_service(EmptyRequest())
#             rospy.loginfo("Next service called successfully.")

#             # Publish the selected button index as an integer
#             self.next_value_publisher.publish(self.selected_button)
#             rospy.loginfo(f"Published selected button index: {self.selected_button}")
#         except rospy.ServiceException as e:
#             rospy.logerr(f"Failed to call next service: {e}")

#         print(f"Button {self.selected_button + 1} selected, proceeding to next step.")
#         self.next_button.config(state=tk.DISABLED)

#     def update_label(self, text):
#         """Update the label with new text."""
#         self.label.config(text=text)

#     def run(self):
#         """Run the Tkinter main loop."""
#         self.root.mainloop()

#     def close(self):
#         """Close the application window."""
#         self.root.destroy()
#         self.root.quit()

class GuiApp:
    def __init__(self, root, next_service, next_value_publisher):
        self.root = root
        self.root.title("ROS Noetic GUI")
        self.root.geometry("1280x720")

        # Set the window to fullscreen
        self.root.attributes('-fullscreen', False)
        # self.root.attributes('-fullscreen', True)

        # Focus the window on startup
        self.root.after(100, self.bring_to_front)

        # Bind the ESC key to exit fullscreen
        self.root.bind("<Escape>", self.exit_fullscreen)

        self.next_service = next_service  # Store the next_service proxy
        self.next_value_publisher = next_value_publisher  # Store the publisher for the next value

        # Upper half text
        self.label = tk.Message(root, text="Waiting for message...", font=("Arial", 16), width=600)
        self.label.pack(pady=20)

        # Container frame for buttons
        self.container_frame = tk.Frame(root)
        self.container_frame.pack(expand=True)  # Center the frame vertically

        # Buttons frame
        self.frame = tk.Frame(self.container_frame)
        self.frame.pack(side=tk.TOP, pady=20)

        self.buttons = []
        self.selected_button = None  # Track the selected button

        # for i in range(5):
        #     btn = tk.Button(self.frame, text=f"Button {i+1}", width=10, height=2,
        #                     command=lambda b=i: self.select_button(b))
        #     btn.pack(side=tk.LEFT, anchor=tk.CENTER, padx=5)
        #     self.buttons.append(btn)
        button_labels = ["Too Slow", "Somewhat Slow", "Not Slow"]

        for i, label in enumerate(button_labels):
            btn = tk.Button(
                self.frame,
                text=label,
                width=20,
                height=2,
                command=lambda b=i: self.select_button(b)
            )
            btn.pack(side=tk.LEFT, anchor=tk.CENTER, padx=5)
            self.buttons.append(btn)

        # Next button
        self.next_button = tk.Button(root, text="Next", width=10, height=2, state=tk.DISABLED, command=self.on_next)
        self.next_button.pack(side=tk.BOTTOM, pady=10)

    def bring_to_front(self):
        """Bring the window to the front and ensure it stays on top."""
        # Force focus on the window
        self.root.focus_force()
        self.root.attributes('-topmost', True)
        self.root.after_idle(self.root.attributes, '-topmost', False)  # Reset -topmost attribute
        
        # Attempt to bring the window to the front using wmctrl (Linux specific)
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

    def select_button(self, button_index):
        """Select a button and enable the Next button."""
        # Reset the previously selected button
        if self.selected_button is not None:
            self.buttons[self.selected_button].config(relief=tk.RAISED)

        # Highlight the selected button
        self.selected_button = button_index
        self.buttons[button_index].config(relief=tk.SUNKEN)

        # Enable the Next button
        self.next_button.config(state=tk.NORMAL)

    def on_next(self):
        """Handle the Next button press."""
        rospy.loginfo("Next button pressed in GUI")
        try:
            self.next_service(EmptyRequest())
            rospy.loginfo("Next service called successfully.")

            # Publish the selected button index as an integer
            self.next_value_publisher.publish(self.selected_button)
            rospy.loginfo(f"Published selected button index: {self.selected_button}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call next service: {e}")

        print(f"Button {self.selected_button + 1} selected, proceeding to next step.")
        self.next_button.config(state=tk.DISABLED)

    def update_label(self, text):
        """Update the label with new text."""
        self.label.config(text=text)

    def run(self):
        """Run the Tkinter main loop."""
        self.root.mainloop()

    def close(self):
        """Close the application window."""
        self.root.destroy()
        self.root.quit()
\



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

    def question_callback(self, msg):
        """Callback to update the GUI label with the latest question."""
        self.latest_question = msg.data
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
        self.gui_app = GuiApp(root, self.next_service, self.next_value_publisher)
        self.gui_app.update_label(self.latest_question)
        self.gui_app.run()

    def run(self):
        """Run the ROS node."""
        rospy.loginfo("GUI Node is running")
        rospy.spin()

if __name__ == '__main__':
    node = GuiNode()
    node.run()
