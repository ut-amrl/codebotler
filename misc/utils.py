import importlib.util
import sys
import pandas as pd
import glob
from pathlib import Path
import json


def custom_serializer(obj):
    if isinstance(obj, (list, tuple)):
        return "__list__", obj
    return obj


def custom_sort_key(file_name):
    # assume the format is in task_prompt_completion.py
    file_name = file_name.split("/")[-1].split(".")[0]
    name, seq1, seq2 = file_name.split("_")
    return name, int(seq1), int(seq2)


def check_file_exists(f: str):
    return Path(f).exists()


def load_module(module_name, module_path):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    foo = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = foo
    spec.loader.exec_module(foo)
    return foo


def read_benchmark(p: Path, read_regex: str):
    if not p.is_dir():
        raise Exception(f"Path {p} is not a directory.")
    python_files = glob.glob(str(p) + "/{}.py".format(read_regex))
    print("benchmark python files: ", python_files)
    benchmarks = []
    for idx, f in enumerate(python_files):
        program = load_module("p{}".format(idx), f)
        benchmark = {
            "name": program.name,
            "prompts": program.prompts,
            "tests": program.tests,
        }
        benchmarks.append(benchmark)
    return pd.DataFrame(benchmarks)


def read_completions(p: Path, read_regex: str):
    # TODO: read completions from file
    if not p.is_dir():
        raise Exception(f"Path {p} is not a directory.")
    python_files = sorted(
        glob.glob(str(p) + "/{}.py".format(read_regex)), key=custom_sort_key
    )

    print("completion python files: ", python_files)
    benchmarks = []
    for idx, f in enumerate(python_files):
        program = load_module("p{}".format(idx), f)
        benchmark = {
            "name": program.name,
            "detailed_name": python_files[idx].split(".")[0].split("/")[-1],
            "prompt": program.prompt,
            "completion": program.completion,
        }
        benchmarks.append(benchmark)
    return pd.DataFrame(benchmarks)


def write_dict_to_python_file(variable_name, data_dict, filename, write_mode="w"):
    # Convert the dictionary to a JSON-formatted string with custom serialization
    json_str = json.dumps(data_dict, default=custom_serializer, indent=4)

    # Create the content of the Python file with the formatted dictionary assignment
    content = f"{variable_name} = {json_str}\n\n"

    # Write the content to the specified filename
    with open(filename, write_mode) as file:
        file.write(content)


def write_custom_to_python_file(message, filename, write_mode="w"):
    # Create the content of the Python file with the formatted dictionary assignment
    content = f"{message}\n\n"

    # Write the content to the specified filename
    with open(filename, write_mode) as file:
        file.write(content)
