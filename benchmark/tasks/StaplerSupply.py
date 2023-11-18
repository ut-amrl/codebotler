import sys

sys.path.insert(0, "../..")
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "StaplerSupply"

prompts = [
    "Check every printer room for a stapler, and come back and tell me which ones do not have a stapler.",
    "Go to every printer room and check to see if there is a stapler. Come back and tell me which printer rooms do not have a stapler.",
    "Go find all printer rooms that do not have a stapler. Come back and tell me.",
    "Check all printer rooms for staplers, and come back and tell me which ones do not have any staplers.",
    "Tell me the name of every printer room that does not have a stapler.",
]

# construct state
state0 = (
    State()
    .addLocation("printer room 1")
    .addLocation("printer room 2")
    .addLocation("printer room 3")
    .addLocation("meeting room")
    .addLocation("start_loc")
    .addObject("stapler", "printer room 1")
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("printer room 1")
    .addLocation("printer room 2")
    .addLocation("printer room 3")
    .addLocation("meeting room")
    .addLocation("start_loc")
    .addObject("stapler", "meeting room")
    .addObject("stapler", "printer room 2")
    .addObject("stapler", "printer room 3")
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("printer room 1")
    .addLocation("printer room 2")
    .addLocation("printer room 3")
    .addLocation("meeting room")
    .addLocation("start_loc")
    .addObject("stapler", "meeting room")
    .addRobotLocation("start_loc")
)

# helper checks
def say_con(trace: Trace, room_num: Union[int, str], VERBOSE: bool = False) -> bool:
    # say printer room room_num does not have a stapler.
    contained_say = [str(room_num), "not|.*no|.*don't|.*doesn't"]

    say_pattern = contain_words(contained_say)

    goto_before_say = (
        trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"printer room 1"))
        and trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"printer room 2"))
        and trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"printer room 3"))
    )

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])

    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug(
            f"say_constraint room num {room_num}", say, [label_S, label_AL, label_IT]
        )
        print_debug(
            f"goto1_before_say room num {room_num}", goto_before_say, [label_EO]
        )

    return goto_before_say and say


def generic_con(trace, VERBOSE: bool = False) -> bool:
    # goes to all printer rooms but not any other room
    go_to_check = (
        trace.Exists(GoTo(r"printer room 1"))
        and trace.Exists(GoTo(r"printer room 2"))
        and trace.Exists(GoTo(r"printer room 3"))
    )

    not_go_to_check = not trace.Exists(GoTo(r"meeting room"))

    # stapler is checked
    stapler_pattern = contain_words(["stapler"])
    check_room_stapler = (
        trace.ActAtFirst(r"printer room 1", CheckEntity(stapler_pattern))
        and trace.ActAtFirst(r"printer room 2", CheckEntity(stapler_pattern))
        and trace.ActAtFirst(r"printer room 3", CheckEntity(stapler_pattern))
    )

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_ES])
        print_debug("not_go_to_check", not_go_to_check, [label_ES])
        print_debug("check_room_stapler", check_room_stapler, [label_CE, label_AL])

    return go_to_check and not_go_to_check and check_room_stapler


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    """
    stapler in printer room 1.
    Say printer rooms 2,3 don't have stapler.
    Don't say printer room 1 has stapler.
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_say_1 = not say_con(trace, 1, VERBOSE)
    say_2 = say_con(trace, 2, VERBOSE)
    say_3 = say_con(trace, 3, VERBOSE)

    return generic and not_say_1 and say_2 and say_3


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    """
    stapler in printer room 2,3.
    Say printer rooms 1 don't have stapler.
    Don't say printer room 2,3 has stapler.
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_1 = say_con(trace, 1, VERBOSE)
    not_say_2 = not say_con(trace, 2, VERBOSE)
    not_say_3 = not say_con(trace, 3, VERBOSE)

    return generic and say_1 and not_say_2 and not_say_3


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    """
    stapler in printer room 2,3.
    Say printer rooms 1,2,3 don't have stapler.
    Don't say printer room 0 has stapler.
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_1 = say_con(trace, 1, VERBOSE)
    say_2 = say_con(trace, 2, VERBOSE)
    say_3 = say_con(trace, 3, VERBOSE)

    return generic and say_1 and say_2 and say_3


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
]
