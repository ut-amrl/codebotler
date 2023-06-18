# Simulator

## Installation

Easiest way is through conda:
`conda create -n potassco -c conda-forge clingo`
`conda activate potassco`

## Walkthrough

- `robot.lp` contains the asp rules governing state changes in our simulated world.
- `solver.py` contains Context, a wrapper around the asp clingo solver. Setting `debug=True` will dump everything passed to a Context object into a file `debug_file="debug.lp"` for debugging.
- `test_solver.py` is a test suite based on real LLM genertaions taken from google doc
- `evaluate.api` takes the jsonl generations and constraints created with `benchmark_generator` and runs them through the asp simulation, outputing `evaluations.jsonl` which appends a "SAT/UNSAT" column reporting success in the simulation.

## Todo

There are still bugs to hunt down. 

I believe I fixed the bug of absolute negation that people were discussing in Slack, i.e. `not go_to(X, T)` not holding for every `room(X)` at time `T`. 

Be on the lookout if it appears again. This is why I added the `debug.lp` file, as it's always a good idea to run `clingo robot.lp debug.lp` to double check the simulation.