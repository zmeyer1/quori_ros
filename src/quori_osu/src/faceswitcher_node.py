#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import cv2
from PIL import Image, ImageTk
import tkinter as tk
import os

class FaceSwitcher:
    def __init__(self):
        # Load the GIFs
        home_dir = os.path.expanduser("~")
        self.default_face_path = os.path.join(home_dir, 'Pictures/default_face.gif')
        self.thinking_face_path = os.path.join(home_dir, 'Pictures/thinking_face.gif')
        self.current_image_path = self.default_face_path

        # Set up Tkinter
        self.root = tk.Tk()
        self.root.attributes('-fullscreen', True)  # Fullscreen mode
        self.root.bind("<Escape>", lambda e: self.root.quit())  # Exit on Escape key

        # Create a label to display the images
        self.label = tk.Label(self.root)
        self.label.pack(expand=True)

        # Set up ROS node
        rospy.init_node('face_switcher_node')

        # Create service servers
        rospy.Service('/default_face', Empty, self.show_default_face)
        rospy.Service('/thinking_face', Empty, self.show_thinking_face)

    def show_default_face(self, req):
        rospy.loginfo("Switching to default face")
        self.current_image_path = self.default_face_path
        self.update_display()
        return []

    def show_thinking_face(self, req):
        rospy.loginfo("Switching to thinking face")
        self.current_image_path = self.thinking_face_path
        self.update_display()
        return []

    def update_display(self):
        # Load the image using PIL
        image = Image.open(self.current_image_path)
        # Convert image to PhotoImage format for Tkinter
        photo = ImageTk.PhotoImage(image)
        self.label.config(image=photo)
        self.label.image = photo  # Keep a reference to avoid garbage collection

    def run(self):
        self.update_display()
        self.root.mainloop()

if __name__ == "__main__":
    face_switcher = FaceSwitcher()
    face_switcher.run()
