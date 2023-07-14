# Benchmark and Robot Simulator

The benchmark tasks are stored in `benchmark.jsonl`. The benchmark works by running a simulation of the LLM-generated code using ASP.

The simulation is checked with ASP temporal constraints for each task. A readable version of constraints can be found in `evaluator/constraints`. An ASP solver (Clingo) is used to determine whether the simulation trace satisfies the constraints.

## Walkthrough

- `evaluator/robot.lp` contains the ASP rules governing state changes in our simulated world.
- `simple_tracer.py` contains a script for turning python generated code into a trace of ASP instructions to feed to the simulation.
- `evaluator/evaluate.py` is called by the top-level RoboEval script and runs the simulation.
- `evaluator/solve_utils.py` contains a class of helper python functions that can be called in ASP.