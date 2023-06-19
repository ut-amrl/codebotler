#!/usr/bin/env python3
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../code_generator"))

from utilities import *
cd_rel(".")

import rospy
from std_msgs.msg import String
import tkinter as tk
from gtts import gTTS
import os
import speech_recognition as sr
import yaml
import time


def listen_for_yes_or_no():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say yes or no!")
        while True:
            audio = r.listen(source)
            try:
                text = r.recognize_google(audio)
                print(f"You said: {text}")
                if "yes" in text.lower():
                    return True

                elif "no" in text.lower():
                    return False
            except sr.UnknownValueError:
                print("Sorry, could not understand audio.")
            except sr.RequestError as e:
                print(f"Speech recognition request failed: {e}")


class MyGUI:
    def __init__(self, master):
        with open('data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)
        self.master = master
        master.title("My GUI")
        master.attributes('-fullscreen', True)  # set full screen mode

        self.button_frame = None

        self.label = tk.Label(master, text="Jackal :)", font=("Helvetica", 120))
        self.label.pack(anchor=tk.CENTER, expand=True)

        rospy.init_node('gui_interface', disable_signals=True)
        self.robot_say_sub = rospy.Subscriber(self.DATA['ROBOT_SAY_TOPIC'], String, self.message_cb)
        self.robot_ask_sub = rospy.Subscriber(self.DATA['ROBOT_ASK_TOPIC'], String, self.ask_cb)
        self.human_response_pub = rospy.Publisher(self.DATA['HUMAN_RESPONSE_TOPIC'], String, queue_size=10)

        self.options = []  # hardcode
        self.button_list = []

    def update_label(self, text):
        # Create a gTTS object and specify the language
        self.label.config(text=f"{text}", font=("Helvetica", 80),
                          wraplength=int(self.master.winfo_screenwidth() * 0.8), justify="center")
        tts = gTTS(text=text, lang='en')
        current_path = os.path.abspath(__file__)

        if not os.path.exists(os.path.join(os.path.dirname(current_path), 'audio')):
            os.makedirs(os.path.join(os.path.dirname(current_path), 'audio'))
        file_path = os.path.join(os.path.dirname(current_path), 'audio', 'hello.mp3')

        # Save the audio file
        tts.save(file_path)

        # Play the audio file
        os.system('mpg321 {}'.format(file_path))

    def message_cb(self, msg):
        print("message:", msg.data)
        self.update_label(msg.data)        
        self.label.config(text="Jackal :)", font=("Helvetica", 120))
        self.label.pack(anchor=tk.CENTER, expand=True)

    def on_button_click(self, option):
        # call another function here
        print("answered {}".format(option))
        for b in self.button_list:
            b.destroy()
        self.button_frame.destroy()
        self.human_response_pub.publish(option)
        self.label.config(text="Jackal :)", font=("Helvetica", 120))
        self.label.pack(anchor=tk.CENTER, expand=True)

    def ask_cb(self, msg):
        msg = eval(msg.data)
        question = msg[-1]
        options = msg[:-1]
        self.update_label(question)

        self.button_frame = tk.Frame(self.master)  # Create the frame here
        self.button_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # create a button with text "Click me!"
        self.button_list.clear()
        for option in options:
            button = tk.Button(self.button_frame, text=option, font=("Helvetica", 40), command=lambda key=option: self.on_button_click(key))
            button.config(width=10, height=10, pady=10)
            button.pack(side="left", fill="x", expand=True)

            self.button_list.append(button)


if __name__ == '__main__':
    print("start gui")
    root = tk.Tk()
    my_gui = MyGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("User shutdown")
        raise
