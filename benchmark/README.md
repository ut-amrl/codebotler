<!-- not finished -->
# Benchmark and Robot Simulator

The benchmark tasks are stored in `benchmarks/{TASKNAME}.py`. The benchmark contains the *prompt*, *initial world states*, and *DSL-specific tests w.r.t the initial world states*.

## Walkthrough

- `simple_tracer.py` contains a script for turning python generated code into a trace of the custom-defined *domain specific language* (DSL).
- `roboeval_dsl.py` contains the implementation of DSL and provides a few examples
