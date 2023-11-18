import sys

sys.path.append("../..")
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
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

# helper checks
def say_con(trace: Trace, status: str, VERBOSE: bool = False) -> bool:
    if status == "warm":
        # 73-99 degrees allowable
        say_pattern = r"\b(?:[8][0-9]|7[3-9])\b"
    elif status == "cold":
        # 40-71 degrees allowable
        say_pattern = r"\b(?:[5-6][0-9]|7[01])\b"

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])

    go_back_to_start = trace.Precedes(GoTo(r"arjun's office"), GoTo("start_loc"))
    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug(f"go_back_to_start status {status}", go_back_to_start, [label_IT])
        print_debug(f"say_constraint status {status}", say, [label_S, label_AL])

    return say and go_back_to_start


def ask_con(trace: Trace, status: str, VERBOSE: bool = False) -> bool:
    # ask at arjun's office
    contain_status = contain_words([status])
    ask_status = trace.ActAtFirst(
        "arjun's office", Ask(contain_status, r".*")
    ) or trace.ActAtFirst("arjun's office", Ask(r".*", contain_status))

    if VERBOSE:
        print_debug("ask_status", ask_status, [label_A, label_AL])

    return ask_status


def generic_con(trace, VERBOSE: bool = False) -> bool:
    # goes to arjun's office but not joydeep's office
    go_to_check = trace.Exists(GoTo(r"arjun's office"))
    not_go_to_check = not trace.Exists(GoTo(r"joydeep's office"))

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_L])
        print_debug("not_go_to_check", not_go_to_check, [label_L])

    return go_to_check and not_go_to_check


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # arjun says warmer
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask = ask_con(trace, "warm", VERBOSE)
    say = say_con(trace, "warm", VERBOSE=VERBOSE)

    return generic and ask and say


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # arjun says colder
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask = ask_con(trace, "cold", VERBOSE)
    say = say_con(trace, "cold", VERBOSE=VERBOSE)

    return generic and ask and say


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
]
