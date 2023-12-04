import sys
sys.path.append('../..')
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "LunchTime"

prompts = [
    "Go to Jill's office and ask her if she'd like to go for lunch tomorrow. If yes, ask her at what time (offer her some reasonable options). Come back and tell me what she said.",
    "Go to Jill's office and ask her whether she's free to go for lunch tomorrow. If yes, ask her when (give her some reasonable times), and come back and tell me what she said.",
    "Visit Jill's office and ask her if she would like to go for lunch tomorrow. If yes, ask her when (offer some reasonable lunch time options). Let me know what she said.",
    "Ask Jill in her office whether she will be free to go for lunch tomorrow. If yes, offer her some reasonable time options to choose from, and come back and tell me what she said.",
    "Ask Jill in her office if she want to go for lunch tomorrow. If yes, give her some reasonable time options and ask her to choose a time. Finally come back and tell me what she said."
]

# construct state
state0 = State().addLocation("jill's office").addLocation("andrew's office").addLocation("start_loc") \
        .addAgent(".*", "jill's office", [r"yes", r".*"]) \
        .addRobotLocation("start_loc")

state1 = State().addLocation("jill's office").addLocation("andrew's office").addLocation("start_loc") \
        .addAgent(".*", "jill's office", [r"no", r".*"]) \
        .addRobotLocation("start_loc")

# helper checks
def say_con(trace: Trace, VERBOSE: bool = False) -> bool:
    goto_before_say = trace.BeforeFirst(Say(".*")).Exists(GoTo(r"jill's office"))

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])
    
    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst("start_loc", Say(r".*"))

    if VERBOSE:
        print_debug("say_constraint", say, [label_S, label_AL])
        print_debug("goto_before_say", goto_before_say, [label_IT])
    return goto_before_say and say

def ask_lunch_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # ask for time
    lunch_pattern = contain_words(["lunch"])
    not_contain = [f"{i}:" for i in range(17,25)] + [f"{i}:" for i in range(4,9)]
    not_contain = ["|.*".join(not_contain)]
    ask_time_pattern = contain_words(["11|.*eleven|.*12|.*twelve"])
    ask_not_contain_time_pattern = contain_words(not_contain)

    ask_time = trace.ActAtFirst(r"jill's office", Ask(ask_time_pattern, r".*")) or \
               trace.ActAtFirst(r"jill's office", Ask(r".*", ask_time_pattern))
    
    not_ask_time = not (trace.ActAtFirst(r"jill's office", Ask(ask_not_contain_time_pattern, r".*")) or \
                        trace.ActAtFirst(r"jill's office", Ask(r".*", ask_not_contain_time_pattern)))

    ask_lunch_before_time = not (trace.BeforeFirst(Ask(lunch_pattern, r".*")).Exists(Ask(ask_time_pattern, r".*")) or \
                                 trace.BeforeFirst(Ask(lunch_pattern, r".*")).Exists(Ask(r".*", ask_time_pattern)))

    if VERBOSE:
        print_debug("ask_time", ask_time, [label_A, label_AL])
        print_debug("ask_lunch_before_time", ask_lunch_before_time, [label_EO])
        print_debug("not_ask_time", not_ask_time, [label_A, label_AL])

    return ask_time and ask_lunch_before_time and not_ask_time

def not_ask_lunch_con(trace: Trace, VERBOSE: bool = False) -> bool:
    lunch_pattern = contain_words(["lunch"])

    only_ask_for_lunc = not trace.Precedes(Ask(lunch_pattern, r".*"), Ask(r".*", r".*")) and \
                        not trace.BeforeFirst(Ask(lunch_pattern, r".*")).Exists(Ask(r".*", r".*"))
    
    if VERBOSE:
        print_debug("only_ask_for_lunc", only_ask_for_lunc, [label_A, label_AL])

    return only_ask_for_lunc

def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # goes to only jill's office and not andrew's office
    go_to_check = trace.Exists(GoTo(r"jill's office")) 
    
    not_go_to_check = not trace.Exists(GoTo(r"andrew's office"))

    # ask for lunch
    lunch_pattern = contain_words(["lunch"])
    ask_lunch = trace.ActAtFirst(r"jill's office", Ask(lunch_pattern, r".*"))

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_L])
        print_debug("not_go_to_check", not_go_to_check, [label_L])
        print_debug("ask_lunch", ask_lunch, [label_A, label_AL])

    return go_to_check and not_go_to_check and ask_lunch

def test0(trace_elements : List[TraceElement], VERBOSE: bool = False) -> bool:
    # jill says yes to lunch
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask = ask_lunch_con(trace, VERBOSE)
    say = say_con(trace, VERBOSE)
    return generic and ask and say

def test1(trace_elements : List[TraceElement], VERBOSE: bool = False) -> bool:
    # jill says no to lunch
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_ask = not_ask_lunch_con(trace, VERBOSE)
    say = say_con(trace, VERBOSE)
    return generic and not_ask and say

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
    }
]