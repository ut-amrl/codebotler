import sys
sys.path.append('../..')
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "SayGoodDay"

prompts = [
    "Go to every lab and if there is a person there, say Good Day!",
    "Visit each lab and if someone is present, greet them with a pleasant Good Day!",
    "Go through all the labs, and if there's a person inside, say 'Good Day!' to them.",
    "Head to each lab and in case there's a person present, bid them a Good Day!",
    "Visit every lab space and when encountering an individual, tell them a Good Day!"
]

# construct state
state0 = State().addLocation("sam's lab").addLocation("amy's lab") \
        .addLocation("lobby").addLocation("conference room").addLocation("start_loc") \
        .addAgent(".*", "sam's lab", []).addAgent(".*", "amy's lab", []) \
        .addRobotLocation("start_loc")

state1 = State().addLocation("sam's lab").addLocation("amy's lab") \
        .addLocation("lobby").addLocation("conference room").addLocation("start_loc") \
        .addAgent(".*", "sam's lab", []) \
        .addRobotLocation("start_loc")

state2 = State().addLocation("sam's lab").addLocation("amy's lab") \
        .addLocation("lobby").addLocation("conference room").addLocation("start_loc") \
        .addAgent(".*", "amy's lab", []) \
        .addRobotLocation("start_loc")

state3 = State().addLocation("sam's lab").addLocation("amy's lab") \
        .addLocation("lobby").addLocation("conference room").addLocation("start_loc") \
        .addAgent(".*", "lobby", []).addAgent(".*", "conference room", []) \
        .addRobotLocation("start_loc")

# helper checks
def say_con(trace: Trace, agent_name: str, VERBOSE: bool = False) -> bool:
    contained_words = ["good", "day"]
    pattern = contain_words(contained_words)
    say = trace.ActAtFirst(f"{agent_name}'s lab", Say(pattern))
    check_before_say = trace.AfterFirst(GoTo(f"{agent_name}'s lab")).BeforeFirst(Say(pattern)).Exists(CheckEntity(r".*"))

    if VERBOSE:
        print_debug("say", say, [label_S, label_AL])
        print_debug("check_before_say", check_before_say, [label_EO])

    return say and check_before_say

def not_say_con(trace: Trace, loc: str, VERBOSE: bool = False) -> bool:
    return not trace.ActAtFirst(loc, Say(r".*"))

def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    go_to_lab = trace.Exists(GoTo(r"sam's lab")) and trace.Exists(GoTo(r"amy's lab"))
    not_go_to_other = not trace.Exists(GoTo(r"lobby")) and not trace.Exists(GoTo(r"conference"))
    check_person_at_lab = trace.ActAtFirst(r"sam's lab", CheckEntity(r".*")) and \
                          trace.ActAtFirst(r"amy's lab", CheckEntity(r".*"))
    
    if VERBOSE:
        print_debug("go_to_lab", go_to_lab, [label_ES])
        print_debug("not_go_to_other", not_go_to_other, [label_ES])
        print_debug("check_person_at_lab", check_person_at_lab, [label_CE, label_AL])

    return go_to_lab and not_go_to_other and check_person_at_lab

def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_sam = say_con(trace, "sam", VERBOSE)
    say_amy = say_con(trace, "amy", VERBOSE)

    return generic and say_sam and say_amy

def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_sam = say_con(trace, "sam", VERBOSE)
    not_say_amy = not_say_con(trace, "amy's lab", VERBOSE)

    return generic and say_sam and not_say_amy

def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_say_sam = not_say_con(trace, "sam's lab", VERBOSE)
    say_amy = say_con(trace, "amy", VERBOSE)

    return generic and not_say_sam and say_amy

def test3(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_say_sam = not_say_con(trace, "sam's lab", VERBOSE)
    not_say_amy = not_say_con(trace, "amy's lab", VERBOSE)

    return generic and not_say_sam and not_say_amy

tests = [
    {
        "state" : state0,
        "test" : test0,
        "timeout": False
    },
    {
        "state" : state1,
        "test" : test1,
        "timeout": False
    },
    {
        "state" : state2,
        "test" : test2,
        "timeout": False
    },
    {
        "state" : state3,
        "test" : test3,
        "timeout": False
    },
]

