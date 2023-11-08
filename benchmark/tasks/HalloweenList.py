import sys

sys.path.insert(0, "../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "HalloweenList"

prompts = [
    "Go to every office, and if there is anyone there, ask if they'd like a chocolate, caramel, or gummy. Come back and tell me how many of each we need to buy.",
    "Go to every office, and if there is a person there, ask them if they'd like a chocolate, caramel, or gummy. Come back and tell me how many of each we need to buy.",
    "Check with every occupied office to see if the occupant would like a chocolate, caramel, or gummy. Let me know how many of each we need to buy.",
    "Find every occupied office and ask their occupants whether they would like a chocolate, caramel, or gummy. Let me know how many of each we need to buy.",
    "Go to every office with a person, and ask them if they would like a chocolate, caramel, or gummy, then come back and tell me how many of each people asked for.",
]

# construct state
state0 = (
    State()
    .addLocation("sally's office")
    .addLocation("mark's office")
    .addLocation("cindy's office")
    .addLocation("kitchen")
    .addLocation("start_loc")
    .addAgent(r".*", "sally's office", ["gumm"])
    .addAgent(r".*", "mark's office", ["caramel"])
    .addAgent(r".*", "cindy's office", ["choco"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("sally's office")
    .addLocation("mark's office")
    .addLocation("cindy's office")
    .addLocation("kitchen")
    .addLocation("start_loc")
    .addAgent(r".*", "sally's office", ["caramel"])
    .addAgent(r".*", "mark's office", ["gumm"])
    .addAgent(r".*", "cindy's office", ["caramel"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("sally's office")
    .addLocation("mark's office")
    .addLocation("cindy's office")
    .addLocation("kitchen")
    .addLocation("start_loc")
    .addAgent(r".*", "sally's office", ["choco"])
    .addAgent(r".*", "mark's office", ["choco"])
    .addRobotLocation("start_loc")
)

# helper checks
def say_con(
    trace: Trace, candy_type: str, candy_num: str, VERBOSE: bool = False
) -> bool:
    # say the choice of candies
    contained_say = [candy_type, candy_num]
    say_pattern = contain_words(contained_say)

    goto1_before_say = trace.BeforeFirst(Say(say_pattern)).Exists(
        GoTo(r"sally's office")
    )
    goto2_before_say = trace.BeforeFirst(Say(say_pattern)).Exists(
        GoTo(r"mark's office")
    )
    goto2_before_say = trace.BeforeFirst(Say(say_pattern)).Exists(
        GoTo(r"cindy's office")
    )

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])
    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug(
            f"say_constraint candy type {candy_type} candy num {candy_num}",
            say,
            [label_S, label_AL],
        )
        print_debug(
            f"goto1_before_say candy type {candy_type} candy num {candy_num}",
            goto1_before_say,
            [label_EO],
        )
        print_debug(
            f"goto2_before_say candy type {candy_type} candy num {candy_num}",
            goto2_before_say,
            [label_EO],
        )
        print_debug(
            f"goto2_before_say candy type {candy_type} candy num {candy_num}",
            goto2_before_say,
            [label_EO],
        )

    return goto1_before_say and goto2_before_say and goto2_before_say and say


def ask_con(
    trace: Trace, person: str, candy_choice: str, VERBOSE: bool = False
) -> bool:
    # ask person if they want chocolate, caramel, or gummy
    contain_candy = contain_words([candy_choice])

    ask_candy = trace.ActAtFirst(
        f"{person}'s office", Ask(contain_candy, r".*")
    ) or trace.ActAtFirst(f"{person}'s office", Ask(".*", contain_candy))

    if VERBOSE:
        print_debug(
            f"person {person} ask_{candy_choice}", ask_candy, [label_A, label_AL]
        )

    return ask_candy


def generic_con(trace, VERBOSE: bool = False) -> bool:
    # goes to only alice and bob's office
    go_to_check = (
        trace.Exists(GoTo(r"sally's office"))
        and trace.Exists(GoTo(r"mark's office"))
        and trace.Exists(GoTo(r"cindy's office"))
    )

    not_go_to_check = not trace.Exists(GoTo(r"kitchen"))

    check_person_in_office = (
        trace.ActAtFirst(r"sally's office", CheckEntity(".*"))
        and trace.ActAtFirst(r"mark's office", CheckEntity(".*"))
        and trace.ActAtFirst(r"cindy's office", CheckEntity(".*"))
    )

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_ES])
        print_debug("not_go_to_check", not_go_to_check, [label_ES])
        print_debug(
            "check_person_in_office", check_person_in_office, [label_CE, label_AL]
        )

    return go_to_check and not_go_to_check and check_person_in_office


def test_0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # go to all offices and get one each
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask_sally = ask_con(trace, "sally", "gumm", VERBOSE)
    ask_mark = ask_con(trace, "mark", "caramel", VERBOSE)
    ask_cindy = ask_con(trace, "cindy", "choco", VERBOSE)

    say_candy = (
        say_con(trace, "choco", "1|.*one", VERBOSE)
        and say_con(trace, "gumm", "1|.*one", VERBOSE)
        and say_con(trace, "caramel", "1|.*one", VERBOSE)
    )
    return generic and ask_sally and ask_mark and ask_cindy and say_candy


def test_1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # go to all offices and get 2 caramel and 1 gummy
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask_sally = ask_con(trace, "sally", "caramel", VERBOSE)
    ask_mark = ask_con(trace, "mark", "gumm", VERBOSE)
    ask_cindy = ask_con(trace, "cindy", "caramel", VERBOSE)

    say_candy = say_con(trace, "gumm", "1|.*one", VERBOSE) and say_con(
        trace, "caramel", "2|.*two", VERBOSE
    )
    return generic and ask_sally and ask_mark and ask_cindy and say_candy


def test_2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # go to sally and mark's office and 2 chocolate
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask_sally = ask_con(trace, "sally", "choco", VERBOSE)
    ask_mark = ask_con(trace, "mark", "choco", VERBOSE)
    not_ask_cindy = not ask_con(trace, "cindy", ".*", VERBOSE)

    say_candy = say_con(trace, "choco", "2|.*two", VERBOSE)
    return generic and ask_sally and ask_mark and not_ask_cindy and say_candy


tests = [
    {"state": state0, "test": test_0, "timeout": False},
    {"state": state1, "test": test_1, "timeout": False},
    {"state": state2, "test": test_2, "timeout": False},
]
