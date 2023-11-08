import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "LunchBreak"

prompts = [
    "Ask if Alice and Bob in their offices are up for lunch. If yes, tell them that we'll meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.",
    "Ask Alice in her office if she is up for lunch, and if yes, tell her that we will meet in the lobby in 5 minutes. Do the same for Bob. Come back and tell me who all are joining for lunch.",
    "Go to Alice's and Bob's offices and ask each of them if they are up for lunch. If they say yes, tell them that we will meet in the lobby in 5 minutes. Come back and tell me who all are joining for lunch.",
    "Ask Alice and Bob if they are up for lunch. If they say yes, tell them that we will meet in the lobby in five minutes. Come back and tell me who all are joining for lunch.",
    "Go to Alice's office and ask her if she is free for lunch. If she says yes, tell her that we will meet in the lobby in five minutes. Do the same for Bob. Come back and tell me who will join for lunch.",
]

# construct state
state0 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["yes"])
    .addAgent(".*", "bob's office", ["yes"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["no"])
    .addAgent(".*", "bob's office", ["no"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["yes"])
    .addAgent(".*", "bob's office", ["no"])
    .addRobotLocation("start_loc")
)

state3 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["no"])
    .addAgent(".*", "bob's office", ["yes"])
    .addRobotLocation("start_loc")
)

# helper checks
def say_con(trace: Trace, person: str, VERBOSE: bool = False) -> bool:
    # say who is coming to lunch
    contained_say = [person]
    excluded_words = ["no", "not"]
    say_pattern = contain_words_and_exlude_words(contained_say, excluded_words)

    goto_before_say = trace.BeforeFirst(Say(say_pattern)).Exists(
        GoTo(r"alice's office")
    ) and trace.BeforeFirst(Say(say_pattern)).Exists(GoTo(r"bob's office"))

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])

    say = trace.AfterFirst(GoTo(exclude_start_loc)).ActAtFirst(
        "start_loc", Say(say_pattern)
    )

    if VERBOSE:
        print_debug(f"say_constraint person {person}", say, [label_S, label_AL])
        print_debug(f"goto_before_say person {person}", goto_before_say, [label_EO])
    return goto_before_say and say


def meet_in_lobby_con(trace: Trace, person: str, VERBOSE: bool = False) -> bool:
    meet_lobby_pattern = contain_words(["5|.*five", "lobby"])
    loc = f"{person}'s office"
    say = trace.ActAtFirst(loc, Say(meet_lobby_pattern))

    lunch_pattern = contain_words(["lunch"])
    ask_before_say = (
        trace.AfterFirst(GoTo(loc))
        .BeforeFirst(GoTo(r"^(?!.*\b{loc}\b).*$".format(loc=loc)))
        .BeforeFirst(Say(meet_lobby_pattern))
        .Exists(Ask(lunch_pattern, r".*"))
    )

    if VERBOSE:
        print_debug(
            f"meet_in_lobby_constraint person {person}", say, [label_S, label_AL]
        )
        print_debug(f"ask_before_say person {person}", ask_before_say, [label_EO])

    return ask_before_say and say


def generic_con(trace, VERBOSE: bool = False) -> bool:
    # goes to only alice and bob's office
    go_to_check = trace.Exists(GoTo(r"alice's office")) and trace.Exists(
        GoTo(r"bob's office")
    )

    not_go_to_check = not trace.Exists(GoTo(r"charlie's office"))

    # ask alice and bob in their office
    lunch_pattern = contain_words(["lunch"])
    ask_alice = trace.ActAtFirst(r"alice's office", Ask(lunch_pattern, r".*"))
    bob_alice = trace.ActAtFirst(r"bob's office", Ask(lunch_pattern, r".*"))

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_L])
        print_debug("not_go_to_check", not_go_to_check, [label_L])
        print_debug("ask_alice", ask_alice, [label_A, label_AL])
        print_debug("bob_alice", bob_alice, [label_A, label_AL])

    return go_to_check and not_go_to_check and ask_alice and bob_alice


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # both alice and bob join
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    alice_meet_lobby = meet_in_lobby_con(trace, "alice", VERBOSE)
    bob_meet_lobby = meet_in_lobby_con(trace, "bob", VERBOSE)
    alice_say = say_con(trace, "alice", VERBOSE)
    bob_say = say_con(trace, "bob", VERBOSE)
    return generic and alice_meet_lobby and bob_meet_lobby and alice_say and bob_say


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # neither join
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_alice_meet_lobby = not meet_in_lobby_con(trace, "alice", VERBOSE)
    not_bob_meet_lobby = not meet_in_lobby_con(trace, "bob", VERBOSE)
    not_alice_say = not say_con(trace, "alice", VERBOSE)
    not_bob_say = not say_con(trace, "bob", VERBOSE)
    return (
        generic
        and not_alice_meet_lobby
        and not_bob_meet_lobby
        and not_alice_say
        and not_bob_say
    )


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # only alice joins
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    alice_meet_lobby = meet_in_lobby_con(trace, "alice", VERBOSE)
    not_bob_meet_lobby = not meet_in_lobby_con(trace, "bob", VERBOSE)
    alice_say = say_con(trace, "alice", VERBOSE)
    not_bob_say = not say_con(trace, "bob", VERBOSE)
    return (
        generic
        and alice_meet_lobby
        and not_bob_meet_lobby
        and alice_say
        and not_bob_say
    )


def test3(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # only bob joins
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_alice_meet_lobby = not meet_in_lobby_con(trace, "alice", VERBOSE)
    bob_meet_lobby = meet_in_lobby_con(trace, "bob", VERBOSE)
    not_alice_say = not say_con(trace, "alice", VERBOSE)
    bob_say = say_con(trace, "bob", VERBOSE)
    return (
        generic
        and not_alice_meet_lobby
        and bob_meet_lobby
        and not_alice_say
        and bob_say
    )


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
    {"state": state3, "test": test3, "timeout": False},
]
