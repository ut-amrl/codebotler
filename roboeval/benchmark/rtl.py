import re
from typing import List, Optional, Union

SPECIAL_DELIM = "/$@!/"


class Trace:
    def __init__(self, traces: List["TraceElement"]):
        self.trace = traces

    def AfterFirst(self, static_check: "StaticCheck") -> "Trace":
        for i, t in enumerate(self.trace):
            if static_check.check_equal(t):
                return Trace(self.trace[i + 1 :])
        # if not found, return self
        return self

    def BeforeFirst(self, static_check: "StaticCheck") -> "Trace":
        for i, t in enumerate(self.trace):
            if static_check.check_equal(t):
                return Trace(self.trace[:i])
        # if not found, return self
        return self

    def AfterLast(self, static_check: "StaticCheck") -> "Trace":
        for i in range(len(self.trace)):
            idx = len(self.trace) - i - 1
            t = self.trace[idx]
            if static_check.check_equal(t):
                return Trace(self.trace[idx + 1 :])
        # if not found, return self
        return self

    def BeforeLast(self, static_check: "StaticCheck") -> "Trace":
        for i in range(len(self.trace)):
            idx = len(self.trace) - i - 1
            t = self.trace[idx]
            if static_check.check_equal(t):
                return Trace(self.trace[:idx])
        # if not found, return self
        return self

    def Exists(self, static_check: "StaticCheck") -> bool:
        for t in self.trace:
            if static_check.check_equal(t):
                return True
        return False

    def Terminal(self, static_check: "StaticCheck") -> bool:
        return static_check.check_equal(self.trace[-1])

    # helper functions
    def Precedes(
        self, static_check1: "StaticCheck", static_check2: "StaticCheck"
    ) -> bool:
        return self.AfterFirst(static_check1).Exists(static_check2)

    def ActAtFirst(self, loc: str, static_check_action: "StaticCheck") -> bool:
        go_to_exist = self.Exists(GoTo(loc))
        act_at = (
            self.AfterFirst(GoTo(loc))
            .BeforeFirst(GoTo(r"^(?!.*\b{loc}\b).*$".format(loc=loc)))
            .Exists(static_check_action)
        )
        return act_at and go_to_exist


class TraceElement:
    def __init__(
        self, element_name: str, value1: str, value2: Union[List[str], None] = None
    ) -> None:
        self.element_name = element_name
        self.value1 = value1
        self.value2 = value2

    def convert_to_str(self) -> str:
        return f"{self.element_name} {SPECIAL_DELIM} {self.value1} {SPECIAL_DELIM} {str(self.value2)}"

    def __str__(self) -> str:
        if self.value2 is not None:
            return (
                "TraceElement: "
                + self.element_name
                + " value: "
                + self.value1
                + " "
                + str(self.value2)
            )
        return "TraceElement: " + self.element_name + " value: " + self.value1


class StaticCheck:
    def __init__(
        self,
        element_name: str,
        regex_value1: str,
        regex_value2: Union[str, None] = None,
    ) -> None:
        self.element_name = element_name
        self.regex_value1 = regex_value1
        self.regex_value2 = regex_value2

    def check_equal(self, other: TraceElement) -> bool:
        if self.element_name != other.element_name:
            return False

        match1 = bool(re.search(self.regex_value1, other.value1, re.IGNORECASE))
        if self.regex_value2 is None:
            return match1

        if other.value2 is None:

            raise Exception(
                f"StaticCheck: other.value2 is None, but self.regex_value2 is not None. \
                            self.name is {self.element_name} and other.name is {other.element_name} "
            )

        for v in other.value2:
            match2 = bool(re.search(self.regex_value2, v, re.IGNORECASE))
            if match2:
                return match1
        return False

    def __str__(self) -> str:
        return (
            "StaticCheck: "
            + self.element_name
            + " value: "
            + self.regex_value1
            + " value 2: "
            + str(self.regex_value2)
        )


# wrapper class
class GoTo(StaticCheck):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("GoTo", regex_value1)


class Say(StaticCheck):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("Say", regex_value1)


class Ask(StaticCheck):
    def __init__(self, regex_value1: str, regex_value2: List[str]) -> None:
        super().__init__("Ask", regex_value1, regex_value2)


class CheckEntity(StaticCheck):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("CheckEntity", regex_value1)


class Pick(StaticCheck):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("Pick", regex_value1)


class Place(StaticCheck):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("Place", regex_value1)


# wrapper class: trace elements
class GoTo_tr(TraceElement):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("GoTo", regex_value1)


class Say_tr(TraceElement):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("Say", regex_value1)


class Ask_tr(TraceElement):
    def __init__(self, regex_value1: str, regex_value2: List[str]) -> None:
        super().__init__("Ask", regex_value1, regex_value2)


class CheckEntity_tr(TraceElement):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("CheckEntity", regex_value1)


class Pick_tr(TraceElement):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("Pick", regex_value1)


class Place_tr(TraceElement):
    def __init__(self, regex_value1: str) -> None:
        super().__init__("Place", regex_value1)


if __name__ == "__main__":
    # example 1
    example_trace = [
        TraceElement("GoTo", "printer room"),
        TraceElement("CheckEntity", "stapler"),
        TraceElement("GoTo", "start loc"),
    ]

    # checks
    trace = Trace(example_trace)
    tc1 = (
        trace.AfterFirst(GoTo("printer room"))
        .BeforeFirst(GoTo(".*"))
        .Exists(CheckEntity("stapler"))
    )

    print("example1: ", tc1)

    # example 2
    example_trace = [
        TraceElement("GoTo", "joydeep's office"),
        TraceElement("GoTo", "joydeep's office"),
    ]

    trace = Trace(example_trace)
    tc1 = trace.AfterFirst(GoTo("joydeep's office")).Exists(GoTo("joydeep's office"))
    print("example2: ", tc1)

    # example 3
    example_trace = [
        TraceElement("GoTo", "X"),
        TraceElement("GoTo", "Y"),
        TraceElement("GoTo", "Z"),
    ]

    trace = Trace(example_trace)

    tc1 = trace.BeforeFirst(GoTo(r"Z")).Exists(GoTo(r"X"))
    tc2 = trace.BeforeFirst(GoTo(r"Z")).Exists(GoTo(r"Y"))
    print("example3: ", tc1 and tc2)

    # example 4
    example_trace = [
        TraceElement("GoTo", "X"),
        TraceElement("CheckEntity", "stapler"),
        TraceElement("GoTo", "Y"),
        TraceElement("CheckEntity", "stapler"),
        TraceElement("GoTo", "Z"),
        TraceElement("CheckEntity", "stapler"),
        TraceElement("CheckEntity", "stapler"),
    ]

    trace = Trace(example_trace)

    tc1 = not trace.AfterFirst(GoTo("Z")).Exists(GoTo(".*"))
    print("example3: ", tc1)

    print(GoTo("X").check_equal(TraceElement("GoTo", "X")))
