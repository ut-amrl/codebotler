import os
import zipfile

from models.model_factory import load_model

from code_generation.completions import completions
from benchmark.simple_tracer import evaluate_trace
from misc.utils import read_benchmark, read_completions

prompt_prefix = ""
prompt_suffix = ""

def generate(args):
  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  model = load_model(args)
  stop_sequences = ["\n#", "\ndef", "\nclass", "```"]
  if args.model_type == "openai-chat":
    prompt_prefix = ""
    prompt_suffix = ""
    stop_sequences = ["\n#", "\nclass", "```"]
  prompts = read_benchmark(args.benchmark_file, args.benchmark_filter)
  completions(
      model,
      stop_sequences,
      args.temperature,
      args.top_p,
      args.max_tokens,
      prompt_prefix,
      prompt_suffix,
      prompts,
      args.generate_output,
      args.num_completions,
  )
  if args.zip_path:
    print(f"Zipping from {args.generate_output} to {args.zip_path}...")
    with zipfile.ZipFile(args.zip_path, "w") as zip_file:
      for file in args.generate_output.glob("*"):
        zip_file.write(file, arcname=file.name)

def evaluate(args):
  print(f"Evaluating completions from {args.generate_output}...")
  print(f"Benchmark file: {args.benchmark_file}")
  if args.unzip_path:
    print(f"Unzipping from {args.unzip_path} to {args.generate_output}...")
    with zipfile.ZipFile(args.unzip_path, "r") as zip_file:
      zip_file.extractall(args.generate_output)
  completions = read_completions(args.generate_output, args.completion_filter)
  benchmarks = read_benchmark(args.benchmark_file, args.benchmark_filter)
  evaluate_trace(completions, benchmarks, args.evaluate_output, args.simulation_timeout)
  
def main():
  global prompt_prefix, prompt_suffix
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument("--generate", action="store_true")
  parser.add_argument("--evaluate", action="store_true")

  parser.add_argument("--generate-output", type=Path)
  parser.add_argument("--zip-path", type=Path)
  parser.add_argument("--evaluate-output", type=Path)
  parser.add_argument("--unzip-path", type=Path)

  parser.add_argument("--model-type", choices=["openai", "openai-chat", "palm", "automodel", "hf-textgen"], default="openai-chat")
  parser.add_argument('--model-name', type=str, help='Model name', default='gpt-4')
  parser.add_argument('--tgi-server-url', type=str, help='Text Generation Inference Client URL', default='http://127.0.0.1:8082')
  parser.add_argument('--chat-prompt-prefix', type=Path, help='Prompt prefix for GPT chat completion only', default='code_generation/openai_chat_completion_prefix.py')
  parser.add_argument('--prompt-prefix', type=Path, help='Prompt prefix', default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=Path, help='Prompt suffix', default='code_generation/prompt_suffix.py')
  parser.add_argument('--max-workers', type=int, help='Maximum number of workers', default=1)

  parser.add_argument('--benchmark-file', type=Path, help='Benchmark file', default='benchmark/tasks/')

  parser.add_argument("--num-completions", type=int, default=20)
  parser.add_argument("--max-tokens", type=int, default=512)
  parser.add_argument("--top-p", type=float, default=0.95)
  parser.add_argument("--temperature", type=float, default=0.2)
  parser.add_argument("--print-completions", action="store_true", help="Print completions to stdout")
  parser.add_argument("--simulation-timeout", type=int, default=1, help="Timeout for simulation in seconds.")
  parser.add_argument("--test-filter", type=str, help="Regex to filter tests to run.")
  parser.add_argument("--benchmark-filter", type=str, default="*", help="Regex to filter which benchmark to run.")
  parser.add_argument("--completion-filter", type=str, default="*", help="Regex to filter which completion to run.")

  # For an automodel
  parser.add_argument("--batch-size", type=int, default=1)
  args = parser.parse_args()

  if not args.generate and not args.evaluate:
    raise ValueError("Must specify either --generate or --evaluate.")
  if args.generate and args.evaluate:
    raise ValueError("Cannot specify both --generate and --evaluate.")
  elif args.generate:
    assert args.generate_output is not None, "Must specify --generate-output with --generate."
  elif args.evaluate:
    assert args.evaluate_output is not None, "Must specify --evaluate-output with --evaluate."
    assert args.generate_output is not None, "Must specify --generate-output with --evaluate."

  if args.generate:
    generate(args)
  elif args.evaluate:
    evaluate(args)

if __name__ == "__main__":
  main()