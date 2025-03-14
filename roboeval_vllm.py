
from misc.utils import read_benchmark, load_module
from benchmark.simple_tracer import evaluate_task
from misc.llm_generation_utils import post_process_vllm_generation

import os
import argparse
from vllm import LLM, SamplingParams
from pathlib import Path
import pandas as pd 
from joblib import Parallel, delayed
from transformers import AutoTokenizer
import json 

os.environ["TOKENIZERS_PARALLELISM"] = "true"

def update_prompt(prompt, tokenizer):
    if args.use_llama3_inst:
        messages = load_module("", "code_generation/openai_chat_completion_prefix.py").__dict__["messages"]
        for msg in messages:
            if msg["role"] == "user":
                msg["content"] = "# Instruction: " + msg["content"]
        messages += [{"role": "user", "content": "# Instruction: " + prompt}]
        prompt = tokenizer.apply_chat_template(messages, tokenize=False)
    else:
        prefix = Path("code_generation/prompt_prefix.py").read_text()
        suffix = Path("code_generation/prompt_suffix.py").read_text()
        prompt = prefix + prompt + suffix 
    return prompt


def get_all_generation(args, tokenizer):
    result = []
    tasknames = []
    completion_num = args.num_completions
    for _, task in BENCHMARK_TASKS.iterrows():
        prompts = task["prompts"]
        tasknames.append(task["name"])
        for prompt in prompts:
            prompt = update_prompt(prompt, tokenizer)
            result.extend([prompt] * completion_num)

    # update stop words
    if args.use_llama3_inst:
        stop_token_ids = [tokenizer.eos_token_id, tokenizer.convert_tokens_to_ids("<|eot_id|>")]   
        sampling_params = SamplingParams(
            temperature=args.temperature, 
            top_p=args.top_p,
            max_tokens=args.max_tokens,
            stop_token_ids=stop_token_ids)      
    else:
        stop_words = ["\n#", "\ndef", "\nclass", "```", "import"]
        sampling_params = SamplingParams(
            temperature=args.temperature, 
            top_p=args.top_p,
            max_tokens=args.max_tokens,
            stop=stop_words)
    return result, tasknames, sampling_params
   
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

    os.makedirs(os.path.join(args.save_dir, args.save_name, "pass1"), exist_ok=True)
    os.makedirs(os.path.join(args.save_dir, args.save_name, "error_breakdown"), exist_ok=True)
    pass1_save_path = os.path.join(args.save_dir, args.save_name, "pass1", "result.csv")
    error_breakdown_save_path = os.path.join(args.save_dir, args.save_name, "error_breakdown", "result.csv")
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
    tokenizer = AutoTokenizer.from_pretrained(args.model_name_or_path)
    prompts, tasknames, sampling_params = get_all_generation(args, tokenizer)
    llm = LLM(model=args.model_name_or_path,
              tensor_parallel_size=args.tensor_parallel_size,
              gpu_memory_utilization=args.gpu_memory_utilization)
    
    outputs = llm.generate(prompts, sampling_params)
    programs = post_process_vllm_generation(outputs)
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
    parser.add_argument("-m", "--model_name_or_path", type=str)
    parser.add_argument("-sd", "--save_dir", type=str, default="eval_results")
    parser.add_argument("-sn", "--save_name", type=str, default="vllm")
    parser.add_argument('--benchmark-file', type=Path, help='Benchmark file', default='benchmark/tasks')
    parser.add_argument("-sp", "--save_program", action="store_true")

    parser.add_argument("--num_completions", type=int, default=20)
    parser.add_argument("--max_tokens", type=int, default=512)
    parser.add_argument("--top_p", type=float, default=0.95)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("-t", "--tensor_parallel_size", type=int, default=1)
    parser.add_argument("-g", "--gpu_memory_utilization", type=float, default=0.7)
    parser.add_argument("--use_llama3_inst", action="store_true")
        
    args = parser.parse_args()
    BENCHMARK_TASKS = read_benchmark(args.benchmark_file, "*")
    PROMPT_VARIATION = 5
    generate_evaluate(args)