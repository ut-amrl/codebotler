<!-- not finished -->
# Benchmark and Robot Simulator

The benchmark tasks are stored in `benchmarks/{TASKNAME}.py`. The benchmark contains the *prompt*, *initial world states*, and *RTL-specific tests w.r.t the initial world states*.

## Walkthrough

- `simple_tracer.py` contains a script for turning python generated code into a trace of the custom-defined *domain specific language* (DSL).
- `rtl.py` contains the implementation of roboeval temporal logic and provides a few examples
- `simulator.py` contains the implementation of the symbolic simulator

## Evaluations
- `evaluations/` directory contains all evaluation results from each LLM's generations 
