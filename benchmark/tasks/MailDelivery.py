import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "MailDelivery"

prompts = [
    "Pick up a mail bin from the mail room. Go to every office and ask if they have any mail they'd like delivered. If so, ask them to put it in the mail bin. Finally, meet me in the mail room with all the mail. Place the mail bin back and tell me which offices they are from.",
    "Go to the mail room to pick a mail bin. Ask at every office if they have any mail to deliver. If they reply yes, ask them to put the mail in the bin. Return to the mail room with the mail. Tell me at the mail room which offices these mail are from and put the mail bin back.",
    "Grab a mail bin from the mail room. Go to every office, and ask if they have any mail to deliver. Ask them to put all mail in the bin if there is any. Put the mail bin back in the mail room and report which offices the mail is from.",
    "Find a mail bin in the mail room and carry it with you. Collect mail from every office by asking if there is any mail they'd like delivered. If yes, ask them to put it in the bin. Return the bin to the mail room and say which offices sent mail.",
    "Take a mail bin from the mail room. For every office, ask if there is any mail to deliver and if someone says yes, ask them to put all mail in the bin. Finally find me at the mail room. Put the mail bin away, and tell me which offices had mail.",
]

# construct state
state0 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("mail room")
    .addLocation("lobby")
    .addLocation("conference room")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["yes", ".*"])
    .addAgent(".*", "bob's office", ["yes", ".*"])
    .addAgent(".*", "charlie's office", ["yes", ".*"])
    .addObject("bin", "mail room")
    .addRobotLocation("start_loc")
)

state1 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("mail room")
    .addLocation("lobby")
    .addLocation("conference room")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["no", ".*"])
    .addAgent(".*", "bob's office", ["yes", ".*"])
    .addAgent(".*", "charlie's office", ["yes", ".*"])
    .addObject("bin", "mail room")
    .addRobotLocation("start_loc")
)

state2 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("mail room")
    .addLocation("lobby")
    .addLocation("conference room")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["yes", ".*"])
    .addAgent(".*", "bob's office", ["no", ".*"])
    .addAgent(".*", "charlie's office", ["no", ".*"])
    .addObject("bin", "mail room")
    .addRobotLocation("start_loc")
)

state3 = (
    State()
    .addLocation("alice's office")
    .addLocation("bob's office")
    .addLocation("charlie's office")
    .addLocation("mail room")
    .addLocation("lobby")
    .addLocation("conference room")
    .addLocation("start_loc")
    .addAgent(".*", "alice's office", ["no", ".*"])
    .addAgent(".*", "bob's office", ["no", ".*"])
    .addAgent(".*", "charlie's office", ["no", ".*"])
    .addObject("bin", "mail room")
    .addRobotLocation("start_loc")
)

# helper checks
def say_con(trace: Trace, person: str, VERBOSE: bool = False) -> bool:
    say = contain_words([person])

    # don't return to start location after leaving start location until end
    say_at_start = trace.AfterFirst(GoTo("mail room")).ActAtFirst("mail room", Say(say))

    if VERBOSE:
        print_debug(
            f"say_constraint person {person}", say_at_start, [label_S, label_AL]
        )

    return say_at_start


def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    go_to_offices_mail_room = (
        trace.Exists(GoTo(r"alice's office"))
        and trace.Exists(GoTo(r"bob's office"))
        and trace.Exists(GoTo(r"charlie's office"))
        and trace.Exists(GoTo(r"mail room"))
    )
    not_go_to_other = not trace.Exists(GoTo(r"lobby")) and not trace.Exists(
        GoTo(r"conference room")
    )

    exclude_start_loc = contain_words_and_exlude_words([], ["start_loc"])
    pick_at_mail_room = trace.ActAtFirst("mail room", Pick("bin"))
    first_mail_room = not trace.BeforeFirst(GoTo("mail room")).Exists(
        GoTo(exclude_start_loc)
    )
    place_at_mail_room = trace.AfterFirst(GoTo("mail room")).ActAtFirst(
        "mail room", Place("bin")
    )
    mail_room_after_visiting_all = (
        trace.AfterFirst(GoTo("mail room"))
        .BeforeFirst(GoTo("mail room"))
        .Exists(GoTo("alice's office"))
        and trace.AfterFirst(GoTo("mail room"))
        .BeforeFirst(GoTo("mail room"))
        .Exists(GoTo("bob's office"))
        and trace.AfterFirst(GoTo("mail room"))
        .BeforeFirst(GoTo("mail room"))
        .Exists(GoTo("charlie's office"))
    )
    manipulate_at_mail = (
        pick_at_mail_room
        and first_mail_room
        and place_at_mail_room
        and mail_room_after_visiting_all
    )

    contained_ask = contain_words(["mail"])
    ask_at_office = (
        trace.ActAtFirst("alice's office", Ask(contained_ask, r".*"))
        and trace.ActAtFirst("bob's office", Ask(contained_ask, r".*"))
        and trace.ActAtFirst("charlie's office", Ask(contained_ask, r".*"))
    )

    if VERBOSE:
        print_debug("go_to_offices_mail_room", go_to_offices_mail_room, [label_ES])
        print_debug("not_go_to_other", not_go_to_other, [label_ES])
        print_debug("ask_at_office", ask_at_office, [label_A, label_AL])
        print_debug("pick_at_mail_room", pick_at_mail_room, [label_M, label_AL])
        print_debug("first_mail_room", first_mail_room, [label_IT])
        print_debug("place_at_mail_room", place_at_mail_room, [label_M, label_AL])
        print_debug(
            "mail_room_after_visiting_all", mail_room_after_visiting_all, [label_IT]
        )

    return (
        go_to_offices_mail_room
        and not_go_to_other
        and ask_at_office
        and manipulate_at_mail
    )


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_alice = say_con(trace, "alice", VERBOSE)
    say_bob = say_con(trace, "bob", VERBOSE)
    say_charlie = say_con(trace, "charlie", VERBOSE)

    return generic and say_alice and say_bob and say_charlie


def test1(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_say_alice = not say_con(trace, "alice", VERBOSE)
    say_bob = say_con(trace, "bob", VERBOSE)
    say_charlie = say_con(trace, "charlie", VERBOSE)
    return generic and not_say_alice and say_bob and say_charlie


def test2(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    say_alice = say_con(trace, "alice", VERBOSE)
    not_say_bob = not say_con(trace, "bob", VERBOSE)
    not_say_charlie = not say_con(trace, "charlie", VERBOSE)

    return generic and say_alice and not_say_bob and not_say_charlie


def test3(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE)
    not_say_alice = not say_con(trace, "alice", VERBOSE)
    not_say_bob = not say_con(trace, "bob", VERBOSE)
    not_say_charlie = not say_con(trace, "charlie", VERBOSE)

    return generic and not_say_alice and not_say_bob and not_say_charlie


tests = [
    {"state": state0, "test": test0, "timeout": False},
    {"state": state1, "test": test1, "timeout": False},
    {"state": state2, "test": test2, "timeout": False},
    {"state": state3, "test": test3, "timeout": False},
]
