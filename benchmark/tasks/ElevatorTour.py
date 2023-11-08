import sys

sys.path.append("../..")

from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List


name = "ElevatorTour"

prompts = [
    "Go to the elevator. Wait until someone shows up and ask them if they are here for the tour. If yes, welcome them to the university, tell them to follow you, and take them to the main conference room. If not, wait for the next person. When you get to the conference room, say you have arrived at the conference room and also say enjoy your visit here!",
    "Go to the elevator, and wait until someone shows up. Ask them if they are here for the tour. If yes, welcome them to the university, tell them to follow you, and take them to the main conference room. If not, wait for the next person. When you get to the conference room, say you have arrived at the conference room and wish them an enjoyable visit.",
    "Go to the elevator and wait for someone to arrive. Inquire if they are here for the tour, and if they say yes, welcome them to the university and tell them to follow you. Then take them to the main conference room. If they say no, wait for someone else. When you get to the conference room, announce your arrival at the conference room and wish them an enjoyable visit.",
    "Go to the elevator and wait for a person to show up. When someone shows up, ask them if they are here for the tour, and if they say yes, welcome them to the university and tell them to follow you. Then take them to the main conference room. If they say no, wait for the next person. When you get to the conference room, announce your arrival at the conference room and wish them an enjoyable visit.",
    "Go to the elevator, wait until someone arrives, and then ask them if they are here for the tour. If they say yes, welcome them to the university and tell them to follow you, and guide them to the main conference room. If they say no, wait for the next person. When you get to the conference room, announce your arrival at the conference room and wish them an enjoyable visit.",
]

# construct state
state0 = (
    State()
    .addLocation("elevator")
    .addLocation("conference room")
    .addLocation("lobby")
    .addLocation("start_loc")
    .addAgent(".*", "elevator", ["yes"])
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("elevator")
    .addLocation("conference room")
    .addLocation("lobby")
    .addLocation("start_loc")
    .addAgent(".*", "elevator", ["no"])
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("elevator")
    .addLocation("conference room")
    .addLocation("lobby")
    .addLocation("start_loc")
    .addAgent(".*", "start_loc", ["yes"])
    .addRobotLocation("start_loc")
)

# helper checks
def answer_yes(trace: Trace, VERBOSE: bool = False) -> bool:
    # say welcome to the university and follow me
    welcome_pattern = contain_words(["welcome", "university"])
    follow_pattern = contain_words(["follow"])
    tour_pattern = contain_words(["tour"])

    say_welcome = trace.ActAtFirst("elevator", Say(welcome_pattern))
    say_follow = trace.ActAtFirst("elevator", Say(follow_pattern))

    loc = "elevator"
    ask_before_say = trace.AfterFirst(GoTo(loc)).BeforeFirst(
        GoTo(r"^(?!.*\b{loc}\b).*$".format(loc=loc))
    ).BeforeFirst(Say(welcome_pattern)).Exists(
        Ask(tour_pattern, r".*")
    ) and trace.AfterFirst(
        GoTo(loc)
    ).BeforeFirst(
        GoTo(r"^(?!.*\b{loc}\b).*$".format(loc=loc))
    ).BeforeFirst(
        Say(follow_pattern)
    ).Exists(
        Ask(tour_pattern, r".*")
    )

    # go to main conference room
    go_to_conf_room = trace.BeforeFirst(GoTo("conference")).Exists(
        Say(welcome_pattern)
    ) and trace.BeforeFirst(GoTo("conference")).Exists(Say(follow_pattern))

    exclude_elevator = contain_words_and_exlude_words([], ["elevator"])
    conf_after_elevator = (
        not trace.AfterFirst(GoTo("elevator"))
        .BeforeFirst(GoTo("conference"))
        .Exists(GoTo(exclude_elevator))
    )

    say_at_conf = trace.ActAtFirst(
        "conference", Say(contain_words(["arrive"]))
    ) and trace.ActAtFirst("conference", Say(contain_words(["enjoy"])))

    if VERBOSE:
        print_debug("say_welcome", say_welcome, [label_S, label_AL])
        print_debug("say_follow", say_follow, [label_S, label_AL])
        print_debug("ask_before_say", ask_before_say, [label_A, label_EO, label_AL])
        print_debug("go_to_conf_room", go_to_conf_room, [label_EO])
        print_debug("conf_after_elevator", conf_after_elevator, [label_EO])
        print_debug("say_at_conf", say_at_conf, [label_S, label_AL])

    return (
        say_welcome
        and say_follow
        and ask_before_say
        and go_to_conf_room
        and conf_after_elevator
        and say_at_conf
    )


def ask_con(trace: Trace, VERBOSE: bool = False) -> bool:
    # ask if person is here for the tour
    tour_pattern = contain_words(["tour"])
    ask = trace.ActAtFirst("elevator", Ask(tour_pattern, r".*"))

    if VERBOSE:
        print_debug("ask", ask, [label_A, label_AL])

    return ask


def generic_con(trace, VERBOSE: bool = False) -> bool:
    # first go to elevator
    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])
    go_to_check = not trace.BeforeFirst(GoTo(r"elevator")).Exists(
        GoTo(exclude_start_loc)
    )
    not_go_to_check = not trace.Exists(GoTo(r"lobby"))

    check_person_at_elevator = trace.ActAtFirst("elevator", CheckEntity(".*"))

    if VERBOSE:
        print_debug("go_to_check", go_to_check, [label_IT])
        print_debug("not_go_to_check", not_go_to_check, [label_L])
        print_debug(
            "check_person_at_elevator", check_person_at_elevator, [label_CE, label_AL]
        )

    return go_to_check and not_go_to_check and check_person_at_elevator


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # person is at elevator and says "yes".
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask = ask_con(trace, VERBOSE)
    answer = answer_yes(trace, VERBOSE)
    return generic and ask and answer


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # person is at elevator and says "no".
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    ask = ask_con(trace, VERBOSE)
    return generic and ask


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    # person is at start_loc.
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_ask = not ask_con(trace, VERBOSE)
    return generic and not_ask


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": True},
    {"state": state2, "test": test2, "timeout": True},
]
