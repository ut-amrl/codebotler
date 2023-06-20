# Simulator

## Installation

Easiest way is through conda:
`conda create -n potassco -c conda-forge clingo`
`conda activate potassco`

## Walkthrough

- `robot.lp` contains the asp rules governing state changes in our simulated world.
- `solver.py` contains Context, a wrapper around the asp clingo solver. Setting `debug=True` will dump everything passed to a Context object into a file `debug_file="debug.lp"` for debugging.
- `test_solver.py` is a test suite based on real LLM genertaions taken from google doc
- `evaluate.api` takes the jsonl generations and constraints created with `benchmark_generator` (or any jsonl in similar format) and runs them through the asp simulation, outputing `evaluations.jsonl` which appends a "SAT/UNSAT" column reporting success in the simulation.

## Updates and Usage

- `solve_utils.py` contains a class of helper python functions that can be called in ASP to check things like `contains(asp_atom, pattern)`. These functions are called in ASP by prepending `@`, eg. `@contains(Atom, Pattern)`.
- we are currently in process of testing if simulator behaves as expected with good/bad handwritten generations. `code_examples` contains per-task json files with the good/bad examples. A script in `code_examples/read_and_dump` reads examples in the format of `staplerSupply.json`, adds the task-specific constraints (stored in a dict) and dumps the result into `code_examples/constrained_examples.jsonl`. The script `code_examples/evaluate_api.py` then runs the simulator on `constrained_examples.jsonl` and outputs `evaluations.jsonl` with the results. It also saves all the generated runs in `debug/debug_ex{i}.lp` so the ASP commands can be inspected further for correctness.

Currently the simulator sets additional examples in `staplerSupply` as `is_sat=true` when they should be `is_sat=false`. This is an issue with how we are writing the good/bad examples and not a simulator issue. Also we need a better way of checking `say()` statements.


## Todo

There are still bugs to hunt down. 

I believe I fixed the bug of absolute negation that people were discussing in Slack, i.e. `not go_to(X, T)` not holding for every `room(X)` at time `T`. 

Be on the lookout if it appears again. This is why I added the `debug.lp` file, as it's always a good idea to run `clingo robot.lp debug.lp` to double check the simulation.