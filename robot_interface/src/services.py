#!/usr/bin/env python3
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../code_generation"))

from utilities import *
add_pythonpath_load_amrl_msgs_cd_rel(".", ".")

from zero_shot_object_detector import GroundingDINO
import rospy
from typing import List
from utilities import process_command_string
import yaml
from amrl_msgs.msg import NavStatusMsg
from amrl_msgs.msg import Localization2DMsg
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time
import torch
import os
import re
from PIL import Image
import shutil


class RobotActions:
    def __init__(self):
        with open('../data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)

        self.last_say_bool = None
        self.available_dsl_fns = ["go_to", "get_current_location", "is_in_room", "say", "get_all_rooms", "ask"]
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../third_party/GroundingDINO", "weights", "groundingdino_swint_ogc.pth")
        self.object_detector_model = GroundingDINO(box_threshold=self.DATA['DINO']['box_threshold'], text_threshold=self.DATA['DINO']['text_threshold'], device=self.device, config_path=config_path, weights_path=weights_path)
        self.latest_image_data = None
        self.current_code_string = None
        self.nav_status = None
        self.current_image_num = 0
        if os.path.exists(os.path.join("..", "images")):
            shutil.rmtree(os.path.join("..", "images"))
        self.new_loc_counter = 0
        self.cur_coords = (None, None, None)  # (x, y, theta)

        # Publishers
        self.nav_goal_pub = rospy.Publisher(self.DATA['NAV_GOAL_TOPIC'], Localization2DMsg, queue_size=1)
        self.robot_say_pub = rospy.Publisher(self.DATA['ROBOT_SAY_TOPIC'], String, queue_size=1)
        self.robot_ask_pub = rospy.Publisher(self.DATA['ROBOT_ASK_TOPIC'], String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/chat_commands', String, self.python_cmds_callback, queue_size=10)
        rospy.Subscriber(self.DATA['LOCALIZATION_TOPIC'], Localization2DMsg, self.localization_callback, queue_size=1)
        rospy.Subscriber(self.DATA['NAV_STATUS_TOPIC'], NavStatusMsg, self.nav_status_callback, queue_size=1)
        rospy.Subscriber(self.DATA['CAM_IMG_TOPIC'], CompressedImage, self.image_callback, queue_size=1, buff_size=2**32)

    def python_cmds_callback(self, msg):
        self.current_code_string, self.last_say_bool = process_command_string(msg.data, self.available_dsl_fns)
        self.execute()

    def nav_status_callback(self, msg):
        self.nav_status = msg.status

    def localization_callback(self, msg):
        self.cur_coords = (msg.pose.x, msg.pose.y, msg.pose.theta)

    def image_callback(self, msg):
        self.latest_image_data = msg.data

    def go_to(self, location) -> None:
        goal_msg = Localization2DMsg()
        if location not in self.DATA['LOCATIONS'][self.DATA['MAP']].keys():
            print(f"Location {location} not found")
            return

        if type(self.cur_coords[0]) != type(None):
            curr_loc = np.array(self.cur_coords)[:2]
            goal_loc = np.array([self.DATA['LOCATIONS'][self.DATA['MAP']][location][0], self.DATA['LOCATIONS'][self.DATA['MAP']][location][1]])
            if np.linalg.norm(curr_loc - goal_loc) < self.DATA['DIST_THRESHOLD']:
                return

        goal_msg.pose.x = self.DATA['LOCATIONS'][self.DATA['MAP']][location][0]
        goal_msg.pose.y = self.DATA['LOCATIONS'][self.DATA['MAP']][location][1]
        goal_msg.pose.theta = self.DATA['LOCATIONS'][self.DATA['MAP']][location][2]
        self.nav_goal_pub.publish(goal_msg)
        time.sleep(0.2)
        while self.nav_status == 0:  # to ensure that the robot has started moving
            time.sleep(0.1)
        while self.nav_status in [2, 3]:  # to ensure that the robot has reached the goal
            time.sleep(0.1)
        time.sleep(1)  # to ensure that the robot has stopped moving

    def _check_and_update_locations(self, new_loc):
        min_dist = np.inf
        closest_loc = None
        for loc, coords in self.DATA['LOCATIONS'][self.DATA['MAP']].items():
            dist = np.sqrt((coords[0] - new_loc[0])**2 + (coords[1] - new_loc[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_loc = loc
        if min_dist <= self.DATA['DIST_THRESHOLD']:
            return closest_loc
        else:
            self.DATA['LOCATIONS'][self.DATA['MAP']][f"NEW_LOC_{self.new_loc_counter}"] = list(new_loc)  # Add new location to the dictionary
            self.new_loc_counter += 1
            return f"NEW_LOC_{self.new_loc_counter-1}"

    def get_current_location(self) -> str:
        return self._check_and_update_locations(self.cur_coords)

    def is_in_room(self, object) -> bool:
        img1 = np.frombuffer(self.latest_image_data, np.uint8)
        img2 = cv2.imdecode(img1, cv2.IMREAD_COLOR)
        img3 = np.array(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
        boxes, logits, phrases, annotated_frame = self.object_detector_model.predict_from_image(img3, object)

        image_dir = os.path.join("..", "images")
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)

        ann_image = Image.fromarray(np.array(annotated_frame).astype(np.uint8))
        ann_image.save(os.path.join(image_dir, f"annotated_frame_{self.current_image_num}.png"))
        self.current_image_num += 1
        return len(boxes) > 0

    def say(self, message) -> None:
        msg = String()
        msg.data = message
        self.robot_say_pub.publish(msg)
        print(f"Robot says: \"{message}\"")
        word_len = len(message.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_SAY'] * word_len * 2)

    def get_all_rooms(self) -> List[str]:
        # TODO: create a separate entry for ROOMS in data.yaml
        return list(self.DATA['LOCATIONS'][self.DATA['MAP']].keys())

    def ask(self, person, question, options=None) -> str:
        response = "no answer"
        if options != None:
            options.append(question)
            msg = String()
            msg.data = str(options)
            self.robot_ask_pub.publish(msg)
            response = rospy.wait_for_message(self.DATA['HUMAN_RESPONSE_TOPIC'], String).data
        if options == None:
            print(f"Robot asks {person}: \"{question}\"")
        else:
            print(f"Robot asks {person}: \"{question}\" with options {options}")
        print(f"Response: {response}")
        word_len = len(question.split(" "))
        time.sleep(self.DATA['SLEEP_AFTER_ASK'] * word_len * 2)
        return response

    def execute(self):
        exec(self.current_code_string)
        if not self.last_say_bool:
            self.say("Task complete!")


if __name__ == "__main__":
    def setup_ros_node():
        rospy.init_node('ros_interface', anonymous=False)
        ra = RobotActions()
        time.sleep(1)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down ROS robot actions node")

    setup_ros_node()
