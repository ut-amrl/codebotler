import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "WeatherPoll"

prompts = [
    "Go to the main entrance, and wait for someone to show up. Ask them if they feel it is hot out. Repeat this until you have 10 responses, and then come back and tell me what percent of people think it is hot out.",
    "Go to the main entrance. When someone shows up, ask them if they feel that it's hot out. Do this until you collect ten people's responses. Come back and tell me which percent of people think it's hot out.",
    "Go wait for someone at the main entrance. Ask them if they feel it is hot out, then wait for the next person and ask the same question. Do this until you have ten responses, then come back and report the percentage of people who think it is hot out.",
    "Go to the main entrance. If there is no one there, wait for someone. Ask them if they feel it is hot out. Ask 10 people this same question until you have 10 replies. Finally, come back and tell me the percentage of people that think it's hot outside.",
    "Go to the main entrance, and if there is no one there, wait for someone to show up. Ask them if they feel it is hot out. Wait for the next person and ask the same question again. Do this 10 times. Come back and report the percentage of people who think it's hot out.",
]

# construct state
state0 = (
    State()
    .addLocation("main entrance")
    .addLocation("main conference room")
    .addLocation("start_loc")
    .addAgent(".*", "main entrance", ["yes"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("main entrance")
    .addLocation("main conference room")
    .addLocation("start_loc")
    .addAgent(".*", "main entrance", ["no"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("main entrance")
    .addLocation("main conference room")
    .addLocation("start_loc")
    .addAgent(
        ".*",
        "main entrance",
        ["yes", "no", "yes", "no", "yes", "yes", "no", "yes", "no", "yes"],
    )
    .addRobotLocation("start_loc")
)

state3 = (
    State()
    .addLocation("main entrance")
    .addLocation("main conference room")
    .addLocation("start_loc")
    .addAgent(
        ".*",
        "main entrance",
        ["no", "yes", "yes", "no", "yes", "no", "no", "yes", "no", "no"],
    )
    .addRobotLocation("start_loc")
)

state4 = (
    State()
    .addLocation("main entrance")
    .addLocation("main conference room")
    .addLocation("start_loc")
    .addRobotLocation("start_loc")
)

# helper checks
def say_con(trace: Trace, percentage: str, VERBOSE: bool = False) -> bool:
    say_pattern = contain_words([percentage])

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])
    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug(
            f"say_constraint percentage {percentage}",
            say,
            [label_S, label_IT, label_AL],
        )

    return say


def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # goes to only entrance and not conference room
    go_to_check = trace.Exists(GoTo(r"entrance"))

    not_go_to_check = not trace.Exists(GoTo(r"conference"))

    # exist 10 checks for person and asks for person
    new_trace_check = trace.AfterFirst(GoTo(r"entrance"))
    new_trace_ask = trace.AfterFirst(GoTo(r"entrance"))
    check_entity_10_times = True
    ask_10_times = True
    for _ in range(10):
        check_entity_10_times = check_entity_10_times and new_trace_check.Exists(
            CheckEntity(r".*")
        )
        new_trace_check = new_trace_check.AfterFirst(CheckEntity(r".*"))

        ask_10_times = ask_10_times and new_trace_ask.Exists(Ask(r".*", r".*"))
        new_trace_ask = new_trace_ask.AfterFirst(Ask(r".*", r".*"))

    ask_10_times = ask_10_times and not new_trace_ask.Exists(Ask(r".*", r".*"))

    # check for person right before asking excepting going to the entrance (staying at the entrance)
    new_trace_ask = trace
    ask_after_check = True
    goto_all_but_entrance = contain_words_and_exlude_words([], ["entrance"])

    for _ in range(10):

        between_trace = new_trace_ask.BeforeFirst(Ask(r".*", r".*")).AfterLast(
            CheckEntity(r".*")
        )

        ask_exist = new_trace_ask.Exists(Ask(r".*", r".*"))
        check_exist_before_ask = new_trace_ask.BeforeFirst(Ask(r".*", r".*")).Exists(
            CheckEntity(r".*")
        )
        check_ask_consecutive = (
            not between_trace.Exists(GoTo(goto_all_but_entrance))
            and not between_trace.Exists(Say(r".*"))
            and not between_trace.Exists(Pick(r".*"))
            and not between_trace.Exists(Place(r".*"))
        )

        ask_after_check = (
            ask_after_check
            and ask_exist
            and check_exist_before_ask
            and check_ask_consecutive
        )

        new_trace_ask = trace.AfterFirst(Ask(r".*", r".*"))

    if VERBOSE:
        print_debug(f"go_to_check", go_to_check, [label_L])
        print_debug(f"not_go_to_check", not_go_to_check, [label_L])
        print_debug(f"check_entity_10_times", check_entity_10_times, [label_ES])
        print_debug(f"ask_10_times", ask_10_times, [label_A, label_ES, label_AL])
        print_debug(f"ask_after_check", ask_after_check, [label_EO])

    return (
        go_to_check
        and not_go_to_check
        and check_entity_10_times
        and ask_10_times
        and ask_after_check
    )


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # all say hot
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = say_con(trace, r"100|.*hundred|.*all", VERBOSE=VERBOSE) or say_con(
        trace, r" 10", VERBOSE=VERBOSE
    )

    return generic and say


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # none says hot
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = say_con(trace, r"0|.*zero|.*no", VERBOSE=VERBOSE)

    return generic and say


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # 60%
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = say_con(trace, r"60|.*sixty|.*6", VERBOSE=VERBOSE) or say_con(
        trace, r" 6", VERBOSE=VERBOSE
    )

    return generic and say


def test3(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # 40%
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say = say_con(trace, r"40|.*forty|.*fourty|\b4\b", VERBOSE=VERBOSE) or say_con(
        trace, r" 4", VERBOSE=VERBOSE
    )

    return generic and say


def test4(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # no person exists so it should timeout
    trace = Trace(trace_elements)
    generic = (
        trace.Exists(GoTo(r"entrance"))
        and not trace.Exists(GoTo(r"conference"))
        and trace.Exists(CheckEntity(r".*"))
        and not trace.Exists(Ask(r".*", r".*"))
        and not trace.Exists(Say(r".*"))
    )

    return generic


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
    {"state": state3, "test": test3, "timeout": False},
    {"state": state4, "test": test4, "timeout": True},
]
