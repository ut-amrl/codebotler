import sys

sys.path.insert(0, "../..")
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "HalloweenShopping"

prompts = [
    "Go to every office, and if there is anyone there, ask them how many from 0 to 4 of the following treats they'd like: chocolate, caramel, and gummy. Come back and tell me how many of each we need to buy.",
    "For each office, check if there is anyone there. If there is, ask the person to choose how many (0-4) of each of the following they'd like: caramel, chocolate, or gummy. Come back and tell me how many of each we need to buy.",
    "Visit every office. If there is a person in the office, ask the person to select 0 to 4 for each of the following categories: gummy, chocolate, or caramel. Return to me and say how many of each we need to buy.",
    "Go to each office. If no person is in the office, skip this office. Otherwise, ask the person in the office to whether they'd like 0,1,2,3, or 4 for each of the following: gummy, caramel, or chocolate. Return to me and say how many of each we need to buy.",
    "Go to every office and skip any office without a person in there. Otherwise, ask the person in the office to decide on how many, from zero to four, of the following they'd like: gummy, caramel, or chocolate. Return to me and say how many of each we need to buy. ",
]

# construct state
state0 = (
    State()
    .addLocation("gavin's office")
    .addLocation("sunny's office")
    .addLocation("frank's office")
    .addLocation("lobby")
    .addLocation("robotics lab")
    .addLocation("start_loc")
    .addAgent(".*", "gavin's office", [r"4|four"])
    .addAgent(".*", "sunny's office", [r"4|four"])
    .addAgent(".*", "frank's office", [r"4|four"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("gavin's office")
    .addLocation("sunny's office")
    .addLocation("frank's office")
    .addLocation("lobby")
    .addLocation("robotics lab")
    .addLocation("start_loc")
    .addAgent(".*", "gavin's office", [r"3|three"])
    .addAgent(".*", "sunny's office", [r"1|one"])
    .addAgent(".*", "frank's office", [r"2|two"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("gavin's office")
    .addLocation("sunny's office")
    .addLocation("frank's office")
    .addLocation("lobby")
    .addLocation("robotics lab")
    .addLocation("start_loc")
    .addAgent(".*", "frank's office", [r"4|four"])
    .addRobotLocation("start_loc")
)

state3 = (
    State()
    .addLocation("gavin's office")
    .addLocation("sunny's office")
    .addLocation("frank's office")
    .addLocation("lobby")
    .addLocation("robotics lab")
    .addLocation("start_loc")
    .addAgent(".*", "frank's office", [r"0|zero"])
    .addRobotLocation("start_loc")
)

state4 = (
    State()
    .addLocation("gavin's office")
    .addLocation("sunny's office")
    .addLocation("frank's office")
    .addLocation("lobby")
    .addLocation("robotics lab")
    .addLocation("start_loc")
    .addRobotLocation("start_loc")
)

# helper checks
def ask_con(trace: Trace, agent_name: str, VERBOSE: bool = False) -> bool:
    contained_chocolate_words = ["choco"]
    contained_caramel_words = ["caramel"]
    contained_gummy_words = ["gumm"]

    chocolate_pattern = contain_words(contained_chocolate_words)
    caramel_pattern = contain_words(contained_caramel_words)
    gummy_pattern = contain_words(contained_gummy_words)

    chocolate_ask = trace.ActAtFirst(
        f"{agent_name}'s office", Ask(chocolate_pattern, r".*")
    )
    caramel_ask = trace.ActAtFirst(
        f"{agent_name}'s office", Ask(caramel_pattern, r".*")
    )
    gummy_ask = trace.ActAtFirst(f"{agent_name}'s office", Ask(gummy_pattern, r".*"))

    if VERBOSE:
        print_debug("chocolate_ask", chocolate_ask, [label_A, label_AL])
        print_debug("caramel_ask", caramel_ask, [label_A, label_AL])
        print_debug("gummy_ask", gummy_ask, [label_A, label_AL])

    return chocolate_ask and caramel_ask and gummy_ask


def say_con(trace: Trace, number: str, item: str, VERBOSE: bool = False) -> bool:
    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])
    say_pattern = contain_words([number, item])

    # don't return to start location after leaving start location until end (needs to check)
    say_at_start = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug("say_at_start", say_at_start, [label_IT])

    return say_at_start


def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    go_to_check = (
        trace.Exists(GoTo(r"gavin's office"))
        and trace.Exists(GoTo(r"sunny's office"))
        and trace.Exists(GoTo(r"frank's office"))
    )

    not_go_to_check = not trace.Exists(GoTo(r"lobby")) and not trace.Exists(
        GoTo(r"robotics")
    )

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_ES])
        print_debug("not_go_to_check", not_go_to_check, [label_ES])

    return go_to_check and not_go_to_check


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_chocolate = say_con(trace, "12", "choco", VERBOSE)
    say_caramel = say_con(trace, "12", "caramel", VERBOSE)
    say_gummy = say_con(trace, "12", "gumm", VERBOSE)

    return generic and say_chocolate and say_caramel and say_gummy


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_chocolate = say_con(trace, "6", "choco", VERBOSE)
    say_caramel = say_con(trace, "6", "caramel", VERBOSE)
    say_gummy = say_con(trace, "6", "gumm", VERBOSE)

    return generic and say_chocolate and say_caramel and say_gummy


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_chocolate = say_con(trace, "4", "choco", VERBOSE)
    say_caramel = say_con(trace, "4", "caramel", VERBOSE)
    say_gummy = say_con(trace, "4", "gumm", VERBOSE)

    return generic and say_chocolate and say_caramel and say_gummy


def test3_and_4(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_chocolate = say_con(trace, "0|.*zero|.*no", "choco", VERBOSE)
    say_caramel = say_con(trace, "0|.*zero|.*no", "caramel", VERBOSE)
    say_gummy = say_con(trace, "0|.*zero|.*no", "gumm", VERBOSE)

    return generic and say_chocolate and say_caramel and say_gummy


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
    {"state": state3, "test": test3_and_4, "timeout": False},
    {"state": state4, "test": test3_and_4, "timeout": False},
]
