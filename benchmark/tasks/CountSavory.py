import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
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

# helper checks
def ask_times_con(trace: Trace, times: int, VERBOSE: bool = False) -> bool:
    ask_sc = Ask(r".*", r".*")

    ask_times = True
    new_trace = trace
    for _ in range(times):
        ask_times = ask_times and new_trace.Exists(ask_sc)
        new_trace = new_trace.AfterFirst(ask_sc)
    ask_times = ask_times and not new_trace.Exists(ask_sc)

    if VERBOSE:
        print_debug("ask_times", ask_times, [label_ES])

    return ask_times


def ask_con(trace: Trace, agent_name: str, food: str, VERBOSE: bool = False) -> bool:
    contained_food = contain_words([food])

    ask_food = trace.ActAtFirst(
        f"{agent_name}'s office", Ask(contained_food, r".*")
    ) or trace.ActAtFirst(f"{agent_name}'s office", Ask(r".*", contained_food))

    # ask after check
    check_before_ask = (
        trace.AfterFirst(GoTo(f"{agent_name}'s office"))
        .BeforeFirst(Ask(contained_food, r".*"))
        .Exists(CheckEntity(r".*"))
    )

    if VERBOSE:
        # print_debug("contained_food", contained_food) # not a bool
        print_debug("ask_food", ask_food, [label_A, label_AL])
        print_debug(
            "check_before_ask", check_before_ask, [label_EO, label_CE, label_AL]
        )

    return ask_food and check_before_ask


def say_con(trace: Trace, count: str, VERBOSE: bool = False) -> bool:
    say_pattern = contain_words([count])

    goto_before_say = (
        trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"alice's office"))
        and trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"bob's office"))
        and trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"charlie's office"))
        and trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"david's office"))
    )

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])

    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug(
            f"say_constraint say_pattern {say_pattern}", say, [label_S, label_AL]
        )
        print_debug(
            f"goto_before_say count {count}", goto_before_say, [label_EO, label_ES]
        )

    return goto_before_say and say


def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # visit all offices
    go_to_check = (
        trace.Exists(GoTo(r"alice's office"))
        and trace.Exists(GoTo(r"bob's office"))
        and trace.Exists(GoTo(r"charlie's office"))
        and trace.Exists(GoTo(r"david's office"))
    )

    not_go_to_check = not trace.Exists(GoTo(r"lobby")) and not trace.Exists(
        GoTo(r"cafeteria")
    )

    # check if someone is there
    check_person = (
        trace.ActAtFirst(r"alice's office", CheckEntity(r".*"))
        and trace.ActAtFirst(r"bob's office", CheckEntity(r".*"))
        and trace.ActAtFirst(r"charlie's office", CheckEntity(r".*"))
        and trace.ActAtFirst(r"david's office", CheckEntity(r".*"))
    )

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_ES])
        print_debug("not_go_to_check", not_go_to_check, [label_ES])
        print_debug("check_person", check_person, [label_CE, label_AL])

    return go_to_check and not_go_to_check and check_person


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 0
    trace = Trace(trace_elements)
    generic = generic_con(trace)
    ask_alice = ask_con(trace, "alice", "donut", VERBOSE)
    ask_bob = ask_con(trace, "bob", "cake", VERBOSE)
    ask_charlie = ask_con(trace, "charlie", "donut", VERBOSE)
    ask_david = ask_con(trace, "david", "cake", VERBOSE)

    say = say_con(trace, "0|.*zero|.*no", VERBOSE)
    ask_times = ask_times_con(trace, 4, VERBOSE)
    return (
        generic
        and ask_alice
        and ask_bob
        and ask_charlie
        and ask_david
        and say
        and ask_times
    )


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 2
    trace = Trace(trace_elements)
    generic = generic_con(trace)
    ask_alice = ask_con(trace, "alice", "sandwich", VERBOSE)
    ask_bob = ask_con(trace, "bob", "donut", VERBOSE)
    ask_charlie = ask_con(trace, "charlie", "cake", VERBOSE)
    ask_david = ask_con(trace, "david", "beef|.*jerky", VERBOSE)

    say = say_con(trace, "2|.*two", VERBOSE)
    ask_times = ask_times_con(trace, 4, VERBOSE)
    return (
        generic
        and ask_alice
        and ask_bob
        and ask_charlie
        and ask_david
        and say
        and ask_times
    )


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 1
    trace = Trace(trace_elements)
    generic = generic_con(trace)
    ask_alice = ask_con(trace, "alice", "beef|.*jerky", VERBOSE)
    ask_bob = ask_con(trace, "bob", "cake", VERBOSE)
    ask_charlie = ask_con(trace, "charlie", "cake", VERBOSE)
    not_ask_david = not ask_con(trace, "david", ".*", VERBOSE)

    say = say_con(trace, "1|.*one", VERBOSE)
    ask_times = ask_times_con(trace, 3, VERBOSE)
    return (
        generic
        and ask_alice
        and ask_bob
        and ask_charlie
        and not_ask_david
        and say
        and ask_times
    )


def test3(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # count: 3
    trace = Trace(trace_elements)
    generic = generic_con(trace)
    not_ask_alice = not ask_con(trace, "alice", ".*", VERBOSE)
    ask_bob = ask_con(trace, "bob", "sandwich", VERBOSE)
    ask_charlie = ask_con(trace, "charlie", "beef|.*jerky", VERBOSE)
    ask_david = ask_con(trace, "david", "sandwich", VERBOSE)

    say = say_con(trace, "3|.*three", VERBOSE)
    ask_times = ask_times_con(trace, 3, VERBOSE)
    return (
        generic
        and not_ask_alice
        and ask_bob
        and ask_charlie
        and ask_david
        and say
        and ask_times
    )


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
    {"state": state3, "test": test3, "timeout": False},
]
