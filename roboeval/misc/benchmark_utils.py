# here are all regex templates
import re
from typing import List

# labels
"""
NOTE: there is often overlap between labels. However, you also often have 2 constraints
with the same overlapping labels eg. constraint_A = (lable_AS, label_IT) and constraint_B = (label_AS, label_IT).
Simply use your judgement to assign the best combination eg. constraint_A = (lable_AS) and constraint_B = (label_IT).

Try to use single labels as much as possible
"""
label_AL = "Action Location"  # UMBRELLA label: checks for action at location eg. exists goto startloc
label_OWK = "Open World Knowledge"
label_M = "Manipulation at Location"
label_EO = "Event Ordering"  # checks for before/after
label_IT = "Correct Initial/Terminal"  # checks for the first or last event in a trace
label_L = "Location"  # simple checks for robot location eg. exists goto startloc
label_ES = "Exhaustive Search"  # "ForAll" style checks
label_A = "Ask Statement at Location"
label_S = "Say Statement at Location"
label_CE = "Check Entity Statements at Location"

labels = [
    label_OWK,
    label_M,
    label_EO,
    label_IT,
    label_L,
    label_ES,
    label_AL,
    label_A,
    label_S,
    label_CE,
]
"""
providing (a|b|c) matches for subword
providing a matches for whole word
"""


def contain_words(words: List[str]) -> str:
    return r"^(?=.*" + r")(?=.*".join(words) + r").*$"


def contain_words_and_exlude_words(
    contained_words: List[str], excluded_words: List[str]
) -> str:
    excluded_pattern = r"|".join(
        r"\b" + re.escape(word) + r"\b" for word in excluded_words
    )
    return (
        r"^(?!.*"
        + excluded_pattern
        + r")(?=.*"
        + r")(?=.*".join(contained_words)
        + r").*$"
    )


def print_debug(check_name: str, check_output: bool, labels: list[str] = []):
    print(f"{check_name}: {check_output}. Labels: {labels}")
