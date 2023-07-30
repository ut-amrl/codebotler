import os
import threading
import json
from code_generation.completions import AutoModel, PaLMModel, OpenAIModel, TextGenerationModel
from code_generation.completions import load_model, completions, read_df
from benchmark.evaluator.evaluate import evaluate_trace

prompt_prefix = ""
prompt_suffix = ""

def generate(args):
  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  model = load_model(args)
  stop_sequences = ["\ndef", "\nclass", "print(", "import "]
  prompts = read_df(args.benchmark_file)
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

def evaluate(args):
  print(f"Evaluating completions from {args.generate_output}...")
  print(f"Benchmark file: {args.benchmark_file}")
  evaluate_trace(args.generate_output, args.evaluate_output, print_completions=args.print_completions, test_filter=args.test_filter)
  

def main():
  global prompt_prefix, prompt_suffix
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument("--generate", action="store_true")
  parser.add_argument("--evaluate", action="store_true")

  parser.add_argument("--generate-output", type=Path)
  parser.add_argument("--evaluate-output", type=Path)

  parser.add_argument("--model-type", choices=["openai", "openai-chat" "palm", "automodel", "hf-textgen"], default="openai")
  parser.add_argument('--model-name', type=str, help='Model name', default='text-davinci-003')
  parser.add_argument('--prompt-prefix', type=Path, help='Prompt prefix', default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=Path, help='Prompt suffix', default='code_generation/prompt_suffix.py')
  parser.add_argument('--max-workers', type=int, help='Maximum number of workers', default=1)

  parser.add_argument('--benchmark-file', type=Path, help='Benchmark file', default='benchmark/benchmark.jsonl')

  parser.add_argument("--num-completions", type=int, default=20)
  parser.add_argument("--max-tokens", type=int, default=256)
  parser.add_argument("--top-p", type=float, default=0.95)
  parser.add_argument("--temperature", type=float, default=0.2)
  parser.add_argument("--print-completions", action="store_true", help="Print completions to stdout")
  parser.add_argument("--test-filter", type=str, help="Regex to filter tests to run.")

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