
from roboeval.misc.utils import read_benchmark, load_module
from roboeval.benchmark.simple_tracer import evaluate_task
from roboeval.misc.llm_generation_utils import post_process_vllm_generation, post_process_program
from roboeval.models.model_factory import load_model
import os
import argparse
from vllm import LLM, SamplingParams
from pathlib import Path
import pandas as pd 
from joblib import Parallel, delayed
from transformers import AutoTokenizer
import json 

os.environ["TOKENIZERS_PARALLELISM"] = "true"

def update_prompt(prompt, tokenizer=None):
    if args.use_llama3_inst or args.model_type == "openai":
        messages = load_module("", "roboeval/code_generation/openai_chat_completion_prefix.py").__dict__["messages"]
        for msg in messages:
            if msg["role"] == "user":
                msg["content"] = "# Instruction: " + msg["content"]
        messages += [{"role": "user", "content": "# Instruction: " + prompt}]
        if tokenizer:
            prompt = tokenizer.apply_chat_template(messages, tokenize=False)
        else:
            prompt = messages
    else:
        prefix = Path("roboeval/code_generation/prompt_prefix.py").read_text()
        suffix = Path("roboeval/code_generation/prompt_suffix.py").read_text()
        prompt = prefix + prompt + suffix 
    
    return prompt


def get_all_generation(args):
    if args.use_llama3_inst:
        tokenizer = AutoTokenizer.from_pretrained(args.model_name_or_path)
        stop_params = {"stop_token_ids": [tokenizer.eos_token_id, tokenizer.convert_tokens_to_ids("<|eot_id|>")]}
    else:
        tokenizer = None
        stop_params = {"stop": ["\n#", "\ndef", "```", "import"]}

    result = []
    tasknames = []
    completion_num = args.num_completions
    for _, task in BENCHMARK_TASKS.iterrows():
        prompts = task["prompts"]
        tasknames.extend([task["name"]] * completion_num * len(prompts))
        assert len(prompts) == PROMPT_VARIATION, f"Prompt variation mismatch: {len(prompts)} vs {PROMPT_VARIATION}"
        for prompt in prompts:
            prompt = update_prompt(prompt, tokenizer)
            result.extend([prompt] * completion_num)

    assert len(result) == len(tasknames), f"Length mismatch: {len(result)} vs {len(tasknames)}"
    return result, tasknames, stop_params
   
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
    tasknames_dedup = tasknames[0:len(tasknames):num_completions_task]
    total_len = len(tasknames_dedup)
    
    results = Parallel(n_jobs=total_len)(delayed(evaluate_task)(
        benchmark_file,
        programs[i*num_completions_task:(i+1)*num_completions_task], 
        tasknames_dedup[i],
        i % PROMPT_VARIATION
        ) for i in range(total_len))
    return results

def generate_evaluate(args):
    prompts, tasknames, stop_params = get_all_generation(args)
    llm = load_model(args)
    
    if args.model_type == "vllm":
        sampling_params = SamplingParams(
            temperature=args.temperature,
            max_tokens=args.max_tokens,
            top_p=args.top_p,
            **stop_params
        )
        outputs = llm.generate(prompts, sampling_params)
        programs = post_process_vllm_generation(outputs)
    elif args.model_type == "gemini":
        unprocessed_programs = llm.generate(prompts, **stop_params, temperature=args.temperature, top_p=args.top_p, max_tokens=args.max_tokens)
        programs = []
        for program in unprocessed_programs:
            program = post_process_program(program)
            programs.append(program)
    elif args.model_type == "openai":
        programs = llm.generate(prompts, **stop_params, temperature=args.temperature, top_p=args.top_p, max_tokens=args.max_tokens)
    else:
        raise ValueError(f"To Be Implemented: {args.model_type}")
    
    results = evaluate(tasknames, programs, args.benchmark_file)
    save_results(results)
    
    program_results = {}
    for i, program in enumerate(programs):
        program_results[tasknames[i] + f"_{i}"] = program
    with open(f"{args.save_dir}/programs.json", "w") as f:
        json.dump(program_results, f)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-mt", "--model_type", type=str, default="vllm")
    parser.add_argument("-m", "--model_name_or_path", type=str)
    parser.add_argument("-sd", "--save_dir", type=str, default="eval_results")
    parser.add_argument("-sn", "--save_name", type=str, default="result.csv")
    parser.add_argument('--benchmark-file', type=Path, help='Benchmark file', default='roboeval/benchmark/tasks')

    parser.add_argument("-num", "--num_completions", type=int, default=20)
    parser.add_argument("-temp", "--temperature", type=float, default=0.2)
    parser.add_argument("--max_tokens", type=int, default=512)
    parser.add_argument("--top_p", type=float, default=0.95)
    parser.add_argument("-tps", "--tensor_parallel_size", type=int, default=1)
    parser.add_argument("-gmu", "--gpu_memory_utilization", type=float, default=0.7)
    parser.add_argument("--use_llama3_inst", action="store_true")
        
    args = parser.parse_args()
    BENCHMARK_TASKS = read_benchmark(args.benchmark_file, "*")
    PROMPT_VARIATION = 5
    generate_evaluate(args)