import sys

sys.path.append("../..")
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import (
    contain_words,
    contain_words_and_exlude_words,
    print_debug,
)
from typing import List

name = "CountSavory"

prompts = [
    "Go to every office, and if there is someone there, ask them whether they'd like a cupcake, ham sandwich, donut, or beef jerky. Come back and tell me how many people chose a savory option.",
    "Visit all offices. If anyone is present; ask them to choose from the options of cupcake, ham sandwich, donut, or beef jerky. Let me know how many people selected a savory option when you return.",
    "Go through each office; if someone is there, ask about their preference among cupcake, donut, beef jerky, or ham sandwich. Report the number of individuals who opted for a savory item after returning.",
    "Go visit each office. If there is a person in the office, inquire about their liking for cupcake, ham sandwich, beef jerky, or donut. Come back and tell me of the count of people who went for a savory option.",
    "Go to every office, and if there is someone there, ask them whether they'd like a beef jerky, cupcake, ham sandwich, or donut. Come back and tell me how many people preferred a savory option.",
]

# construct state
state0 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("david's office")
    .addLocation("lobby")
    .addLocation("cafeteria")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", [r"donut"])
    .addAgent(".*", "bob's office", [r"cake"])
    .addAgent(".*", "charlie's office", [r"donut"])
    .addAgent(".*", "david's office", [r"cake"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("david's office")
    .addLocation("lobby")
    .addLocation("cafeteria")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", [r"sandwich"])
    .addAgent(".*", "bob's office", [r"donut"])
    .addAgent(".*", "charlie's office", [r"cake"])
    .addAgent(".*", "david's office", [r"beef|jerky"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("david's office")
    .addLocation("lobby")
    .addLocation("cafeteria")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", [r"beef|jerky"])
    .addAgent(".*", "bob's office", [r"cake"])
    .addAgent(".*", "charlie's office", [r"cake"])
    .addRobotLocation("start_loc")
)

state3 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("david's office")
    .addLocation("lobby")
    .addLocation("cafeteria")
    .addLocation("start_loc")
    .addAgent(".*", "bob's office", [r"sandwich"])
    .addAgent(".*", "charlie's office", [r"beef|jerky"])
    .addAgent(".*", "david's office", [r"sandwich"])
    .addRobotLocation("start_loc")
)


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 0
    trace = Trace(trace_elements)
    say_pattern = contain_words(["0|.*zero|.*no"])
    say = trace.Exists(Say(say_pattern))

    if VERBOSE:
        print_debug("say", say)

    return say


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 2
    trace = Trace(trace_elements)
    say_pattern = contain_words(["2|.*two"])
    say = trace.Exists(Say(say_pattern))

    if VERBOSE:
        print_debug("say", say)

    return say


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 1
    trace = Trace(trace_elements)
    say_pattern = contain_words(["1|.*one"])
    say = trace.Exists(Say(say_pattern))

    if VERBOSE:
        print_debug("say", say)

    return say


def test3(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 3
    trace = Trace(trace_elements)
    say_pattern = contain_words(["3|.*three"])
    say = trace.Exists(Say(say_pattern))

    if VERBOSE:
        print_debug("say", say)

    return say


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
    {"state": state3, "test": test3, "timeout": False},
]
