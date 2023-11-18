import sys

sys.path.insert(0, "../..")
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List


name = "MovieMessenger"

prompts = [
    "Ask Sally in her office if she wants to go to the cinema with Mark. Go to Mark's office and tell him Sally's answer. If Sally says yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM - then go tell Sally what time Mark is leaving.",
    "Ask Sally if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if Sally said yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM, and then go tell Sally what time Mark is leaving.",
    "Ask Sally if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM. If Sally said yes, go tell Sally what time Mark is leaving.",
    "Go to Sally's office and ask her if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM, and then go back to Sally's office and tell her what Mark said.",
    "Ask Sally if she wants to go to the cinema with Mark. Tell Mark what Sally said, and if yes, ask Mark whether he wants to leave at 4PM, 5PM, or 6PM. Then then go tell Sally what Mark said.",
]

# construct state
state0 = (
    State()
    .addLocation("sally's office")
    .addLocation("mark's office")
    .addLocation("start_loc")
    .addAgent(".*", "sally's office", ["yes"])
    .addAgent(".*", "mark's office", ["6pm"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("sally's office")
    .addLocation("mark's office")
    .addLocation("start_loc")
    .addAgent(".*", "sally's office", ["yes"])
    .addAgent(".*", "mark's office", ["5pm"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("sally's office")
    .addLocation("mark's office")
    .addLocation("start_loc")
    .addAgent(".*", "sally's office", ["no"])
    .addAgent(".*", "mark's office", [".*"])
    .addRobotLocation("start_loc")
)

# helper checks
def answer_no_con(trace: Trace, VERBOSE: bool = False) -> bool:
    new_trace = trace.AfterFirst(GoTo("mark's office")).AfterFirst(
        Say(contain_words(["no"]))
    )

    not_ask_say = not new_trace.Exists(Ask(r".*", r".*")) and not new_trace.Exists(
        Say(r".*")
    )

    if VERBOSE:
        print_debug("not_ask_say", not_ask_say, [label_A, label_AL])

    return not_ask_say


def answer_yes_con(trace: Trace, time: str, VERBOSE: bool = False) -> bool:
    ask_mark = trace.ActAtFirst(
        r"mark's office", Ask(contain_words([time]), r".*")
    ) or trace.ActAtFirst(r"mark's office", Ask(r".*", contain_words([time])))

    goto_sally_after_mark = trace.AfterFirst(GoTo(r"mark's office")).Exists(
        GoTo(r"sally's office")
    ) and not trace.AfterFirst(GoTo(r"mark's office")).BeforeFirst(
        GoTo(r"sally's office")
    ).Exists(
        GoTo(r"start_loc")
    )

    say_time_sally = trace.AfterFirst(GoTo(r"mark's office")).ActAtFirst(
        r"sally's office", Say(contain_words([time]))
    )
    if VERBOSE:
        print_debug("ask_mark", ask_mark, [label_A, label_AL])
        print_debug("goto_sally_after_mark", goto_sally_after_mark, [label_EO])
        print_debug("say_time_sally", say_time_sally, [label_S, label_AL])

    return ask_mark and goto_sally_after_mark and say_time_sally


def say_sally_decision_con(trace: Trace, decision: str, VERBOSE: bool = False) -> bool:
    sally_decision = trace.ActAtFirst("mark's office", Say(contain_words([decision])))
    say_before_ask = (
        not trace.AfterFirst(GoTo("mark's office"))
        .BeforeFirst(Say(contain_words([decision])))
        .Exists(Ask(r".*", r".*"))
    )

    if VERBOSE:
        print_debug("sally_decision", sally_decision, [label_S, label_AL])
        print_debug("say_before_ask", say_before_ask, [label_EO])

    return sally_decision and say_before_ask


def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # goes to only alice and bob's office
    go_to_check = trace.Exists(GoTo(r"sally's office")) and trace.Exists(
        GoTo(r"mark's office")
    )

    go_to_sally_first = not trace.BeforeFirst(GoTo(r"sally's office")).Exists(
        GoTo(r"mark's office")
    )

    go_sally_to_mark = (
        not trace.AfterFirst(GoTo(r"sally's office"))
        .BeforeFirst(GoTo(r"mark's office"))
        .Exists(GoTo(r"start_loc"))
    )

    ask_pattern = contain_words(["cinema|.*film|.*movie", "mark"])
    check_person_in_office = trace.ActAtFirst(
        r"sally's office", Ask(ask_pattern, r".*")
    )

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_L])
        print_debug("go_to_sally_first", go_to_sally_first, [label_IT])
        print_debug("go_sally_to_mark", go_sally_to_mark, [label_EO])
        print_debug(
            "check_person_in_office", check_person_in_office, [label_CE, label_AL]
        )

    return (
        go_to_check
        and go_to_sally_first
        and go_sally_to_mark
        and check_person_in_office
    )


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # sally said yes and mark said 6pm
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    sally_yes = say_sally_decision_con(trace, "yes", VERBOSE)
    mark_6pm = answer_yes_con(trace, "6pm", VERBOSE)
    return generic and sally_yes and mark_6pm


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # sally said yes and mark said 5pm
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    sally_yes = say_sally_decision_con(trace, "yes", VERBOSE)
    mark_5pm = answer_yes_con(trace, "5pm", VERBOSE)
    return generic and sally_yes and mark_5pm


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # sally said no
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    sally_no = say_sally_decision_con(trace, "no", VERBOSE) and answer_no_con(
        trace, VERBOSE
    )
    return generic and sally_no


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
]
