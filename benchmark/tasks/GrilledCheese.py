import sys
sys.path.append('../..')
from benchmark.rtl import *
from benchmark.simulator import State
from misc.benchmark_utils import *
from typing import List

name = "GrilledCheese"

prompts = [
    "Compile a list of ingredients for grilled cheese. Go to Zarko's office, and ask him which of them he has. Come back and tell me what he does not have.",
    "Create a list of ingredients needed to make grilled cheese. Visit Zarko's office and inquire which of them he possesses. Return and inform me of the items he does not have.",
    "Put together a compilation of grilled cheese ingredients. Head to Zarko's office, inquire about his available ingredients, and then update me on what he does not have.",
    "Compile the necessary ingredients for grilled cheese. Go to Zarko's office, ask him about his available items, and then report back to me which items he does not have.",
    "Put together a list of grilled cheese ingredients. Head to Zarko's office and inquire about his available ingredients. Return and inform me of what he is missing."
]

# construct state
state0 = State().addLocation("lobby").addLocation("zarko's office").addLocation("start_loc") \
        .addAgent(".*", "zarko's office", [r".*"]) \
        .addRobotLocation("start_loc")

# helper checks
def generic_con(trace: Trace, VERBOSE: bool = False) -> bool:
    contain_cheese = contain_words(["cheese"])
    contain_bread = contain_words(["bread"])

    go_to_check = not trace.Exists(GoTo(r"lobby")) and trace.Exists(GoTo(r"zarko's office"))
    ask_at_zarko_cheese = trace.ActAtFirst("zarko's office", Ask(r".*", contain_cheese)) or \
                          trace.ActAtFirst("zarko's office", Ask(contain_cheese, r".*"))
    ask_at_zarko_bread = trace.ActAtFirst("zarko's office", Ask(r".*", contain_bread)) or \
                         trace.ActAtFirst("zarko's office", Ask(contain_bread, r".*"))
    say_at_start = trace.AfterFirst(GoTo(r"zarko's office")).ActAtFirst("start_loc", Say(r".*"))

    if VERBOSE:
        print("==== generic_con ====")
        print_debug("go_to_check", go_to_check, [label_L])
        print_debug("ask_at_zarko_cheese", ask_at_zarko_cheese, [label_OWK, label_A, label_AL])
        print_debug("ask_at_zarko_bread", ask_at_zarko_bread, [label_OWK, label_A, label_AL])
        print_debug("say_at_start", say_at_start, [label_IT, label_S, label_AL])

    return go_to_check and ask_at_zarko_cheese and ask_at_zarko_bread and say_at_start

def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool: 
    trace = Trace(trace_elements)
    generic = generic_con(trace, VERBOSE=VERBOSE)

    return generic

tests = [
    {
        "state" : state0,
        "test" : test0,
        "timeout": False
    }
]

