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

name = "LunchTime"

prompts = [
    "Go to Jill's office and ask her if she'd like to go for lunch tomorrow. If yes, ask her at what time (offer her some reasonable options). Come back and tell me what she said.",
    "Go to Jill's office and ask her whether she's free to go for lunch tomorrow. If yes, ask her when (give her some reasonable times), and come back and tell me what she said.",
    "Visit Jill's office and ask her if she would like to go for lunch tomorrow. If yes, ask her when (offer some reasonable lunch time options). Let me know what she said.",
    "Ask Jill in her office whether she will be free to go for lunch tomorrow. If yes, offer her some reasonable time options to choose from, and come back and tell me what she said.",
    "Ask Jill in her office if she want to go for lunch tomorrow. If yes, give her some reasonable time options and ask her to choose a time. Finally come back and tell me what she said.",
]

# construct state
state0 = (
    State()
    .addLocation("aiden's office")
    .addLocation("andrew's office")
    .addLocation("start_loc")
    .addAgent(".*", "aiden's office", [r"yes", r".*"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("aiden's office")
    .addLocation("andrew's office")
    .addLocation("start_loc")
    .addAgent(".*", "aiden's office", [r"no", r".*"])
    .addRobotLocation("start_loc")
)


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # aiden says yes to lunch
    trace = Trace(trace_elements)

    ask_time_pattern = contain_words(["11|.*eleven|.*12|.*twelve"])
    not_contain = [f"{i}:" for i in range(17, 25)] + [f"{i}:" for i in range(4, 9)]
    not_contain = ["|.*".join(not_contain)]
    ask_not_contain_time_pattern = contain_words(not_contain)

    ask = trace.Exists(Ask(r".*", ask_time_pattern)) or trace.Exists(
        Ask(ask_time_pattern, r".*")
    )
    not_ask = not (
        trace.Exists(Ask(r".*", ask_not_contain_time_pattern))
        or trace.Exists(Ask(ask_not_contain_time_pattern, r".*"))
    )

    if VERBOSE:
        print_debug("ask_time_pattern", ask_time_pattern)
        print_debug("ask", ask)
        print_debug("not_ask", not_ask)

    return ask and not_ask


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # aiden says no to lunch
    return True


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
]
