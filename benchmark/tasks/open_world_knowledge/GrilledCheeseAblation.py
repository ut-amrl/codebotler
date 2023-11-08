import sys

sys.path.append("../..")
from benchmark.roboeval_dsl import *
from benchmark.simulator import State
from misc.benchmark_utils import (
    contain_words_and_exlude_words,
    contain_words,
    print_debug,
)
from typing import List

name = "GrilledCheese"

prompts = [
    "Compile a list of ingredients for grilled cheese. Go to Zarko's office, and ask him which of them he has. Come back and tell me what he does not have.",
    "Create a list of ingredients needed to make grilled cheese. Visit Zarko's office and inquire which of them he possesses. Return and inform me of the items he does not have.",
    "Put together a compilation of grilled cheese ingredients. Head to Zarko's office, inquire about his available ingredients, and then update me on what he does not have.",
    "Compile the necessary ingredients for grilled cheese. Go to Zarko's office, ask him about his available items, and then report back to me which items he does not have.",
    "Put together a list of grilled cheese ingredients. Head to Zarko's office and inquire about his available ingredients. Return and inform me of what he is missing.",
]

# construct state
state0 = (
    State()
    .addLocation("lobby")
    .addLocation("zarko's office")
    .addLocation("start_loc")
    .addAgent(".*", "zarko's office", [r".*"])
    .addRobotLocation("start_loc")
)


def test0(trace_elements: List[TraceElement], VERBOSE: bool = False) -> bool:
    trace = Trace(trace_elements)

    contain_cheese = contain_words(["cheese"])
    contain_bread = contain_words(["bread"])
    ask_cheese = trace.Exists(Ask(r".*", contain_cheese)) or trace.Exists(
        Ask(contain_cheese, r".*")
    )
    ask_bread = trace.Exists(Ask(r".*", contain_bread)) or trace.Exists(
        Ask(contain_bread, r".*")
    )

    return ask_cheese and ask_bread


tests = [{"state": state0, "test": test0, "timeout": False}]
