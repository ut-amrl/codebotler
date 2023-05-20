import os
import sys
import roslib


def add_pythonpath_load_amrl_msgs_cd(add_rel_path, cd_rel_path):
    sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), add_rel_path))
    os.system(f"cd {os.path.join(os.path.dirname(os.path.realpath(__file__)), cd_rel_path)}")
    roslib.load_manifest('amrl_msgs')

def add_pythonpath(add_rel_path):
    sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), add_rel_path))
    
def cd_rel(cd_rel_path):
    os.system(f"cd {os.path.join(os.path.dirname(os.path.realpath(__file__)), cd_rel_path)}")

def load_amrl_msgs():
    roslib.load_manifest('amrl_msgs')