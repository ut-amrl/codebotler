
from misc.utils import read_benchmark, load_module
from benchmark.simple_tracer import evaluate_task
from models.OpenAIChatModel import OpenAIChatModel
from misc.llm_generation_utils import post_process_vllm_generation

import os
import argparse
from pathlib import Path
import pandas as pd 
from joblib import Parallel, delayed
import json 

os.environ["TOKENIZERS_PARALLELISM"] = "true"

def update_prompt(prompt):
    messages = load_module("", "code_generation/openai_chat_completion_prefix.py").__dict__["messages"]
    for msg in messages:
        if msg["role"] == "user":
            msg["content"] = "# Instruction: " + msg["content"]
    messages += [{"role": "user", "content": "# Instruction: " + prompt}]
    return messages

def get_all_generation(args):
    result = []
    tasknames = []
    completion_num = args.num_completions
    for _, task in BENCHMARK_TASKS.iterrows():
        prompts = task["prompts"]
        tasknames.extend([task["name"]] * completion_num * PROMPT_VARIATION)
        for prompt in prompts:
            prompt = update_prompt(prompt)
            result.extend([prompt] * completion_num)

    # update stop words
    stop_words = ["\n#", "\ndef", "```", "import"]
    return result, tasknames, stop_words
   
def save_results(results):    
    pass1_result = {}
    error_breakdown = {
        "RobotExecutionError": 0,
        "PythonError": 0,
        "CompletionError": 0,
        "Success": 0
    }
    for result in results:
        for key, value in result.items():
            if key == "error_names":
                for error_name in value:
                    error_breakdown[error_name] += 1
            else:
                pass1_result[key] = value

    os.makedirs(os.path.join(args.save_dir, "pass1"), exist_ok=True)
    os.makedirs(os.path.join(args.save_dir, "error_breakdown"), exist_ok=True)
    pass1_save_path = os.path.join(args.save_dir, "pass1", args.save_name)
    error_breakdown_save_path = os.path.join(args.save_dir, "error_breakdown", args.save_name)
    pd.DataFrame(pass1_result.items(), columns=["name", "pass_1"]).to_csv(pass1_save_path, index=False)
    pd.DataFrame(error_breakdown.items(), columns=["name", "error_count"]).to_csv(error_breakdown_save_path, index=False)

def evaluate(tasknames, programs, benchmark_file):
    # use joblib to speed up evaluation process
    num_completions_task = args.num_completions
    total_len = len(tasknames) * PROMPT_VARIATION
    
    results = Parallel(n_jobs=total_len)(delayed(evaluate_task)(
        benchmark_file,
        programs[i*num_completions_task:(i+1)*num_completions_task], 
        tasknames[int(i//PROMPT_VARIATION)],
        i % PROMPT_VARIATION
        ) for i in range(total_len))
    return results

def generate_evaluate(args):
    prompts, tasknames, stop_words = get_all_generation(args)

    # If there exists a ".openai_api_key" file, use that as the API key.
    if os.path.exists(".openai_api_key"):
      with open(".openai_api_key", "r") as f:
        openai_api_key = f.read().strip()
    else:
      openai_api_key = os.getenv("OPENAI_API_KEY")
    assert len(openai_api_key) > 0, \
        "OpenAI API key not found. " + \
        "Either create a '.openai_api_key' file or " + \
        "set the OPENAI_API_KEY environment variable."
    llm = OpenAIChatModel(model=args.model_name, api_key=openai_api_key)    
    programs = llm.generate(prompts, stop_words, args.temperature, args.top_p, args.max_tokens)
    os.makedirs("eval/openai_gpt4o/programs", exist_ok=True)
    program_results = {}
    for i, program in enumerate(programs):
        program_results[tasknames[i] + f"_{i}"] = program
    with open("eval/openai_gpt4o/programs/programs.json", "w") as f:
        json.dump(program_results, f)
    results = evaluate(tasknames, programs, args.benchmark_file)
    save_results(results)
    
    if args.save_program:
        program_results = {}
        for i, program in enumerate(programs):
            program_results[tasknames[int(i//5)] + f"_{i}"] = program
        with open(f"{args.save_dir}/{args.save_name}/programs.json", "w") as f:
            json.dump(program_results, f)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--model_name", type=str, default="gpt-4o-mini")
    parser.add_argument("-sd", "--save_dir", type=str, default="eval_results")
    parser.add_argument("-sn", "--save_name", type=str, default="result.csv")
    parser.add_argument('--benchmark-file', type=Path, help='Benchmark file', default='benchmark/tasks')
    parser.add_argument("-sp", "--save_program", action="store_true")

    parser.add_argument("--num_completions", type=int, default=20)
    parser.add_argument("--max_tokens", type=int, default=512)
    parser.add_argument("--top_p", type=float, default=0.95)
    parser.add_argument("--temperature", type=float, default=0.2)
    
    args = parser.parse_args()
    BENCHMARK_TASKS = read_benchmark(args.benchmark_file, "*")
    PROMPT_VARIATION = 5
    generate_evaluate(args)