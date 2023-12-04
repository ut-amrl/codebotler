import sys
sys.path.append('../..')
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "GetDrink"

prompts = [
    "Go to the lobby, and ask if the visitor would like a bottle of water, iced tea, or lemonade. Bring what they ask for from the kitchen.",
    "Ask any visitor in the lobby if they would like a bottle of water, iced tea, or lemonade. Bring what they ask for from the kitchen.",
    "Go to the lobby. Ask if the visitor wants: a bottle of water, iced tea, or lemonade. Go to the kitchen and bring the visitor back what they asked for.",
    "Go to the lobby and ask any person there if they want iced tea, lemonade or a bottle of water. Bring whatever they asked for back from the kitchen.",
    "Go to the lobby, and ask if the visitor would like a bottle of water, iced tea, or lemonade. If they want a bottle of water, bring it to them from the kitchen. If they want iced tea, bring it to them from the kitchen. If they want lemonade, bring it to them from the kitchen."
]

# construct state
state0 = State().addLocation("lobby").addLocation("kitchen").addLocation("office").addLocation("start_loc") \
        .addAgent(".*", "lobby", ["water"]) \
        .addObject("water", "kitchen").addObject("tea", "kitchen").addObject("lemonade", "kitchen").addObject("drink", "kitchen") \
        .addRobotLocation("start_loc")

state1 = State().addLocation("lobby").addLocation("kitchen").addLocation("office").addLocation("start_loc") \
        .addAgent(".*", "lobby", ["tea"]) \
        .addObject("water", "kitchen").addObject("tea", "kitchen").addObject("lemonade", "kitchen").addObject("drink", "kitchen") \
        .addRobotLocation("start_loc")

state2 = State().addLocation("lobby").addLocation("kitchen").addLocation("office").addLocation("start_loc") \
        .addAgent(".*", "lobby", ["lemonade"]) \
        .addObject("water", "kitchen").addObject("tea", "kitchen").addObject("lemonade", "kitchen").addObject("drink", "kitchen") \
        .addRobotLocation("start_loc")

# helper checks
def pick_place_con(trace: Trace, drink_type: str, VERBOSE: bool = False) -> bool:
    contain_drink = contain_words([drink_type])
    pick_drink_at_kitchen = trace.ActAtFirst(r"kitchen", Pick(contain_drink))

    if VERBOSE:
        print_debug(f"pick_drink_at_kitchen drink type {drink_type}", 
                    pick_drink_at_kitchen, [label_M, label_AL])

    return pick_drink_at_kitchen

def ask_con(trace: Trace, drink_type: str, VERBOSE: bool = False) -> bool:
    contain_drink = contain_words([drink_type])

    ask_drink = trace.ActAtFirst(r"lobby", Ask(contain_drink, r".*")) or \
                trace.ActAtFirst(r"lobby", Ask(r".*", contain_drink)) 
    
    if VERBOSE:
        print_debug(f"ask_constraint drink type {drink_type}", ask_drink, [label_A, label_AL])

    return ask_drink

def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # goes to only lobby and kitchen
    go_to_check = trace.Exists(GoTo(r"lobby")) and trace.Exists(GoTo(r"kitchen"))
    not_go_to_check = not trace.Exists(GoTo(r"office"))

    # go to lobby first and then kitchen
    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])

    goto_lobby_first = not trace.BeforeFirst(GoTo(r"lobby")).Exists(GoTo(exclude_start_loc))
    kitchen_after_lobby = trace.Precedes(GoTo(r"lobby"), GoTo(r"kitchen"))
    lobby_after_kitchen = trace.Precedes(GoTo(r"kitchen"), GoTo(r"lobby"))
    no_goto_start_between = not trace.AfterFirst(GoTo(r"lobby")).BeforeLast(GoTo(r"lobby")).Exists(GoTo(r"start_loc"))
    go_to_order_check = goto_lobby_first and kitchen_after_lobby and lobby_after_kitchen and no_goto_start_between

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_L])
        print_debug("not_go_to_check", not_go_to_check, [label_L])
        print_debug("goto_lobby_first", goto_lobby_first, [label_IT])
        print_debug("kitchen_after_lobby", kitchen_after_lobby, [label_EO])
        print_debug("lobby_after_kitchen", lobby_after_kitchen, [label_EO])
        print_debug("no_goto_start_between", no_goto_start_between, [label_EO])

    return go_to_check and not_go_to_check and go_to_order_check

def test0(trace_elements : List[TraceElement], VERBOSE: bool = False) -> bool:
    # ask for water
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = ask_con(trace, "water", VERBOSE=VERBOSE)
    pick_place = pick_place_con(trace, "water", VERBOSE=VERBOSE)

    return generic and say and pick_place

def test1(trace_elements : List[TraceElement], VERBOSE: bool = False) -> bool:
    # ask for tea
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = ask_con(trace, "tea", VERBOSE=VERBOSE)
    pick_place = pick_place_con(trace, "tea", VERBOSE=VERBOSE)

    return generic and say and pick_place

def test2(trace_elements : List[TraceElement], VERBOSE: bool = False) -> bool:
    # ask for lemonade
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = ask_con(trace, "lemonade", VERBOSE=VERBOSE)
    pick_place = pick_place_con(trace, "lemonade", VERBOSE=VERBOSE)

    return generic and say and pick_place

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
