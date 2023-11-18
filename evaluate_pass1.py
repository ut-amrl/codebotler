from misc.utils import load_module, read_benchmark
from pathlib import Path
import numpy as np 
import argparse

def get_tasknames(BENCHMARKS):
    return BENCHMARKS.name.tolist()

def get_test_num(taskname, BENCHMARKS):
    return len(BENCHMARKS[BENCHMARKS["name"] == taskname].iloc[0]["tests"])

def get_module_data(llm_name):
    module = load_module(llm_name, f"benchmark/evaluations/{llm_name}.py")

    data = module.__dict__
    return data

def evaluate_states(module_dict, taskname, prompt_idx, test_num, completion_idx):
    is_sat = True
    for t in range(test_num):
        field_name = f"{taskname}_{prompt_idx}_{completion_idx}_{t}"
        is_sat = is_sat and eval(module_dict[field_name]['is_sat'])
        if not is_sat:
            break
    return is_sat

def evaluate_execution_error(module_dict, taskname, prompt_idx, test_num, completion_idx):
    execution_error = False
    for t in range(test_num):
        field_name = f"{taskname}_{prompt_idx}_{completion_idx}_{t}"
        execution_error = execution_error or module_dict[field_name]["error_message"] != ""
        if execution_error:
            break
    return execution_error


def compute_pass1(module_dict, taskname, prompt_idx, benchmarks, completion_num=20):
    test_num = get_test_num(taskname, benchmarks)

    correct_num = 0
    for c in range(completion_num):
        is_sat = evaluate_states(module_dict, taskname, prompt_idx, test_num, c)

        if is_sat:
            correct_num += 1
    return correct_num / completion_num


def compute_task_pass1(module_dict, task_name, benchmarks, prompt_num=5, completion_num=20):
    cdf_list = []
    for prompt_idx in range(prompt_num):
        pass1 = compute_pass1(module_dict, task_name, prompt_idx, benchmarks, completion_num=completion_num)
        cdf_list.append(pass1)
    
    return cdf_list

def auto_compute_completion_num(llm):
    module_dict = get_module_data(llm)
    taskname = module_dict["ordered_field_buffer"][0]["name"].split("_")[:-2]
    taskname = "_".join(taskname)
    for i in range(len(module_dict["ordered_field_buffer"])):
        tmp_taskname = module_dict["ordered_field_buffer"][i]["name"].split("_")[:-2]
        tmp_taskname = "_".join(tmp_taskname)
        if tmp_taskname != taskname:
            return int(module_dict["ordered_field_buffer"][i-1]["name"].split("_")[-2]) + 1

def pretty_print(args):
    benchmarks = read_benchmark(Path("benchmark/tasks/"), "*")
    completion_num = auto_compute_completion_num(args.llm)
    module_dict = get_module_data(args.llm)
    
    if "all" in args.tasks:
        task_names = sorted(get_tasknames(benchmarks))
    else:
        task_names = args.tasks

    print("Model: ", args.llm)
    for taskname in task_names:
        cdf = compute_task_pass1(module_dict, taskname, benchmarks, prompt_num=5, completion_num=completion_num)
        print(taskname, end=" ")
        print("pass@1: ", cdf)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--llm", type=str, default="codellama34")
    parser.add_argument("--tasks", type=str, nargs='+', default=["all"])

    args = parser.parse_args()
    pretty_print(args)
    