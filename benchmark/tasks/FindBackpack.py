import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "FindBackpack"

prompts = [
    "Check where I left my backpack in all of the conference rooms and bring it back to me.",
    "Search all the conference rooms for my backpack and return it to me.",
    "Please find my backpack in each conference room and then bring it to me.",
    "Go through every conference room and find my backpack, then bring it back to me.",
    "Look in all of the conference rooms to search for my backpack and then bring it back to me.",
]

# construct state
state0 = (
    State()
    .addLocation("conference room 1")
    .addLocation("conference room 2")
    .addLocation("conference room 3")
    .addLocation("tom's office")
    .addLocation("lobby")
    .addLocation("start_loc")
    .addObject("backpack", "conference room 1")
    .addObject("laptop", "conference room 1")
    .addObject("chair", "conference room 2")
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("conference room 1")
    .addLocation("conference room 2")
    .addLocation("conference room 3")
    .addLocation("tom's office")
    .addLocation("lobby")
    .addLocation("start_loc")
    .addObject("backpack", "conference room 2")
    .addObject("laptop", "conference room 1")
    .addObject("chair", "conference room 2")
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("conference room 1")
    .addLocation("conference room 2")
    .addLocation("conference room 3")
    .addLocation("tom's office")
    .addLocation("lobby")
    .addLocation("start_loc")
    .addObject("backpack", "conference room 3")
    .addObject("laptop", "conference room 1")
    .addObject("chair", "conference room 2")
    .addRobotLocation("start_loc")
)

# helper checks
def pick_con(trace: Trace, loc: str, VERBOSE: bool = False) -> bool:
    # pick backpack at loc
    new_trace = trace
    pick_backpack = new_trace.ActAtFirst(loc, Pick("backpack"))
    while new_trace.AfterFirst(GoTo(loc)).Exists(GoTo(loc)):
        new_trace = new_trace.AfterFirst(GoTo(loc))
        pick_backpack = pick_backpack or new_trace.ActAtFirst(loc, Pick("backpack"))

    # there exist only one pick
    pick_once = not trace.AfterFirst(Pick(r".*")).Exists(Pick(r".*"))

    # check if there is a backpack in every conference room upon visiting the room
    check_backpack = trace.ActAtFirst(loc, CheckEntity(r"backpack"))

    if VERBOSE:
        print_debug("pick_backpack", pick_backpack, [label_M, label_AL])
        print_debug("pick_once", pick_once, [label_M, label_AL])
        print_debug("check_backpack", check_backpack, [label_CE, label_AL])

    return pick_backpack and pick_once and check_backpack


def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])

    not_go_to_other = not trace.Exists(GoTo(r"lobby")) and not trace.Exists(
        GoTo(r"tom's office")
    )
    bring_backpack = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Place(r"backpack")
    ) or not trace.Exists(Place(r"backpack"))

    # exist pick and place
    exist_pick = (
        trace.Exists(Pick(r"backpack"))
        and not trace.Exists(Place(r"laptop"))
        and not trace.Exists(Place(r"chair"))
    )

    if VERBOSE:
        print_debug("not_go_to_other", not_go_to_other, [label_ES])
        print_debug("bring_backpack", bring_backpack, [label_M, label_AL])
        print_debug("exist_pick_place", exist_pick, [label_M, label_AL])

    return not_go_to_other and bring_backpack and exist_pick


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    pick_conf1 = pick_con(trace, r"conference room 1", VERBOSE)

    return generic and pick_conf1


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    pick_conf2 = pick_con(trace, r"conference room 2", VERBOSE)

    return generic and pick_conf2


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    pick_conf3 = pick_con(trace, r"conference room 3", VERBOSE)

    return generic and pick_conf3


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
]
