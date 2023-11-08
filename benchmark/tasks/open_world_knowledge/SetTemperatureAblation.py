import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import (
    contain_words,
    contain_words_and_exlude_words,
    print_debug,
)
from typing import List

name = "SetTemperature"

prompts = [
    "The thermostat is set to 72 degrees. Go to Arjun's office and ask him if he'd like it to be warmer or colder. Come back and tell me what temperature I should set it to.",
    "Ask Arjun in his office if he would like it to be warmer or colder. The thermostat is currently set to 72 degrees. Come back and tell me what temperature I should set the thermostat to based on what Arjun says.",
    "The thermostat is currently set to 72 degrees. Go ask if Arjun thinks his office should be colder or warmer. Come back and tell me what temperature in degrees I should change the thermostat to.",
    "Go to Arjun's office and ask him if he would like his office to be warmer or colder. The thermostat says the temperature in the office is currently 72 degrees. Come back and tell me what temperature I should reset it to based on Arjun's preferences.",
    "The thermostat is set to 72 degrees. Go to Arjun's office and ask if he would like the temperature to be set to a warmer or colder value. Return and tell me what temperature in degrees I should set the thermostat to.",
]

# construct state
state0 = (
    State()
    .addLocation("arjun's office")
    .addLocation("joydeep's office")
    .addLocation("start_loc")
    .addAgent(".*", "arjun's office", [r"warm"])
    .addAgent(".*", "joydeep's office", [r"cold"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("arjun's office")
    .addLocation("joydeep's office")
    .addLocation("start_loc")
    .addAgent(".*", "arjun's office", [r"cold"])
    .addAgent(".*", "joydeep's office", [r"warm"])
    .addRobotLocation("start_loc")
)


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # arjun says warmer
    trace = Trace(trace_elements)

    say_pattern = r"\b(?:[8][0-9]|7[3-9])\b"
    say = trace.Exists(Say(say_pattern))

    if VERBOSE:
        print_debug("say", say)

    return say


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # arjun says colder
    trace = Trace(trace_elements)

    say_pattern = r"\b(?:[5-6][0-9]|7[01])\b"
    say = trace.Exists(Say(say_pattern))

    if VERBOSE:
        print_debug("say", say)

    return say


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
]
