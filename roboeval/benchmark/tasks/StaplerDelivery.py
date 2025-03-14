import sys
sys.path.append('../..')
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "StaplerDelivery"

prompts = [
    "Check every printer room for a stapler, and deliver a stapler from the supply room to every printer room without a stapler.",
    "Go to every printer room and check if there is a stapler. For each visited room that does not have a stapler, deliver a stapler from the supply room to that room.",
    "Check if there is a stapler in every printer room. For each visited room without a stapler, deliver a stapler from the supply room to that room.",
    "Make sure there is a stapler in each printer room. Otherwise, bring a stapler from the supply room to each printer room that currently lacks one.",
    "Check if every printer room has a stapler. If not, put a stapler from the supply room to every printer room without a stapler."
]

# construct state
state0 = State().addLocation("printer room 1").addLocation("printer room 2").addLocation("printer room 3").addLocation("supply room") \
        .addLocation("meeting room").addLocation("start_loc") \
        .addObject("stapler", "supply room").addObject("stapler", "supply room").addObject("stapler", "supply room") \
        .addObject("stapler", "printer room 1") \
        .addRobotLocation("start_loc")

state1 = State().addLocation("printer room 1").addLocation("printer room 2").addLocation("printer room 3").addLocation("supply room") \
        .addLocation("meeting room").addLocation("start_loc") \
        .addObject("stapler", "supply room").addObject("stapler", "supply room").addObject("stapler", "supply room") \
        .addObject("stapler", "printer room 2").addObject("stapler", "printer room 3") \
        .addRobotLocation("start_loc")

state2 = State().addLocation("printer room 1").addLocation("printer room 2").addLocation("printer room 3").addLocation("supply room") \
        .addLocation("meeting room").addLocation("start_loc") \
        .addObject("stapler", "supply room").addObject("stapler", "supply room").addObject("stapler", "supply room") \
        .addRobotLocation("start_loc")

# helper checks
def manipulation_con(trace: Trace, room_nums: List[int], VERBOSE: bool = False) -> bool:
    # carry stapler from supply room to printer room room_num
    n_pick = True
    new_trace = trace
    for _ in range(len(room_nums)):
        n_pick = n_pick and new_trace.ActAtFirst("supply room", Pick("stapler"))
        new_trace = new_trace.AfterFirst(Pick("stapler"))
    n_pick = not new_trace.Exists(Pick("stapler")) and n_pick

    # we are not checking if CheckEntity is before Place for benchmarking simplicity here
    n_place = True
    for room_num in room_nums:
        new_trace = trace
        place_at_printer = new_trace.ActAtFirst(f"printer room {room_num}", Place("stapler"))
        while new_trace.AfterFirst(GoTo(f"printer room {room_num}")).Exists(GoTo(f"printer room {room_num}")):
            new_trace = new_trace.AfterFirst(GoTo(f"printer room {room_num}"))
            place_at_printer = place_at_printer or new_trace.ActAtFirst(f"printer room {room_num}", Place("stapler"))

        n_place = place_at_printer and n_place

    not_pick_at_printer_room = not trace.ActAtFirst("printer room", Pick("stapler"))

    if VERBOSE:
        print_debug("n_pick", n_pick, [label_M, label_AL])
        print_debug("n_place", n_place, [label_M, label_AL])
        print_debug("not_pick_at_printer_room", not_pick_at_printer_room, [label_M, label_AL])

    return n_pick and n_place and not_pick_at_printer_room

def generic_con(trace, VERBOSE: bool = False) -> bool:
    # goes to all printer rooms but not any other room
    go_to_check = trace.Exists(GoTo(r"printer room 1")) and \
                  trace.Exists(GoTo(r"printer room 2")) and \
                  trace.Exists(GoTo(r"printer room 3"))
    
    not_go_to_check = not trace.Exists(GoTo(r"meeting room"))

    # stapler is checked
    stapler_pattern = contain_words(["stapler"])
    check_room_stapler = trace.ActAtFirst(r"printer room 1", CheckEntity(stapler_pattern)) and \
                         trace.ActAtFirst(r"printer room 2", CheckEntity(stapler_pattern)) and \
                         trace.ActAtFirst(r"printer room 3", CheckEntity(stapler_pattern))

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_ES])
        print_debug("not_go_to_check", not_go_to_check, [label_ES])
        print_debug("check_room_stapler", check_room_stapler, [label_CE, label_AL])

    return go_to_check and not_go_to_check and check_room_stapler

def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    """
    stapler in printer room 1. 
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    manipulation = manipulation_con(trace, [2,3], VERBOSE)

    return generic and manipulation

def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    """
    stapler in printer room 1. 
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    manipulation = manipulation_con(trace, [2,3], VERBOSE) # no need to check not place at 1 because only 2 picks
    
    return generic and manipulation

def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    """
    stapler in printer room 1. 
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    manipulation = manipulation_con(trace, [1], VERBOSE)

    return generic and manipulation

def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    """
    stapler in printer room 1. 
    """
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    manipulation = manipulation_con(trace, [1,2,3], VERBOSE)

    return generic and manipulation

tests = [
    {
        "state": state0,
        "test": test0,
        "timeout": False
    },
    {
        "state": state1,
        "test": test1,
        "timeout": False
    },
    {
        "state": state2,
        "test": test2,
        "timeout": False
    }
]