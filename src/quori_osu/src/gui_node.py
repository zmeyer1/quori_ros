#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from threading import Thread
import tkinter as tk
from std_msgs.msg import String

class GuiApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Noetic GUI")
        self.root.geometry("640x480")
        
        # Upper half text
        # self.label = tk.Label(root, text="Waiting for message...", font=("Arial", 16))
        self.label = tk.Message(root, text="Waiting for message...", font=("Arial", 16), width=600)
        self.label.pack(pady=20)
        
        # Buttons frame
        self.frame = tk.Frame(root)
        self.frame.pack(side=tk.TOP, pady=20)
        
        self.buttons = []
        self.selected_button = None  # Track the selected button
        
        for i in range(5):
            btn = tk.Button(self.frame, text=f"Button {i+1}", width=10, height=2,
                            command=lambda b=i: self.select_button(b))
            btn.pack(side=tk.LEFT, anchor=tk.CENTER, padx=5)
            self.buttons.append(btn)
        
        # Next button
        self.next_button = tk.Button(root, text="Next", width=10, height=2, state=tk.DISABLED, command=self.on_next)
        self.next_button.pack(side=tk.BOTTOM, pady=10)

    def select_button(self, button_index):
        # Reset the previously selected button
        if self.selected_button is not None:
            self.buttons[self.selected_button].config(relief=tk.RAISED)
        
        # Highlight the selected button
        self.selected_button = button_index
        self.buttons[button_index].config(relief=tk.SUNKEN)
        
        # Enable the Next button
        self.next_button.config(state=tk.NORMAL)

    def on_next(self):
        # Implement what happens when the Next button is clicked
        print(f"Button {self.selected_button + 1} selected, proceeding to next step.")
        self.next_button.config(state=tk.DISABLED)

    def update_label(self, text):
        self.label.config(text=text)

    def run(self):
        self.root.mainloop()

    def close(self):
        # self.root.after(0, self.root.quit)
        # self.root.after(0, self.root.destroy())

        # TODO: Implement the right way to close the GUI
        self.root.destroy()
        self.root.quit()

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

    def question_callback(self, msg):
        self.latest_question = msg.data
        if self.gui_app is not None:
            self.gui_app.update_label(self.latest_question)

    def start_gui(self, req):
        if self.gui_app is None:
            rospy.loginfo("Starting GUI")
            self.gui_thread = Thread(target=self.launch_gui)
            self.gui_thread.start()
        else:
            rospy.logwarn("GUI is already running")
        return EmptyResponse()

    def stop_gui(self, req):
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
        root = tk.Tk()
        self.gui_app = GuiApp(root)
        self.gui_app.update_label(self.latest_question)
        self.gui_app.run()

    def run(self):
        rospy.loginfo("GUI Node is running")
        rospy.spin()

if __name__ == '__main__':
    node = GuiNode()
    node.run()
