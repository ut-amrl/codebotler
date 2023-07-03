import os
import threading
import json
from code_generation.completions import AutoModel, PaLMModel, OpenAIModel, TextGenerationModel
from code_generation.completions import completions, read_df
from benchmark.evaluator.evaluate import evaluate_trace

model = None
prompt_prefix = ""
prompt_suffix = ""

def load_model(args):
  global model
  if args.model_type == "openai":
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
    model = OpenAIModel(model=args.model_name, api_key = openai_api_key)
  elif args.model_type == "palm":
    # If there exists a ".palm_api_key" file, use that as the API key.
    if os.path.exists(".palm_api_key"):
      with open(".palm_api_key", "r") as f:
        palm_api_key = f.read().strip()
    else:
      palm_api_key = os.getenv("PALM_API_KEY")
    assert len(palm_api_key) > 0, \
        "PaLM API key not found. " + \
        "Either create a '.palm_api_key' file or " + \
        "set the PALM_API_KEY environment variable."
    model = PaLMModel(model=args.model_name, api_key = palm_api_key)
  elif args.model_type == "automodel":
    model = AutoModel(batch_size=1, path=args.model_name)
  elif args.model_type == "hf-textgen":
    model = TextGenerationModel(args.model_name, args.max_workers)
  else:
    raise ValueError(f"Unknown model type: {args.model_type}")

def generate(args):
  prompt_prefix = args.prompt_prefix.read_text()
  prompt_suffix = args.prompt_suffix.read_text()
  load_model(args)
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
  evaluate_trace(args.generate_output, args.evaluate_output)
  

def main():
  global prompt_prefix, prompt_suffix
  import argparse
  from pathlib import Path
  parser = argparse.ArgumentParser()

  parser.add_argument("--generate", action="store_true")
  parser.add_argument("--evaluate", action="store_true")

  parser.add_argument("--generate-output", type=Path)
  parser.add_argument("--evaluate-output", type=Path)

  parser.add_argument("--model-type", choices=["openai", "palm", "automodel", "hf-textgen"], default="openai")
  parser.add_argument('--model-name', type=str, help='Model name', default='text-davinci-003')
  parser.add_argument('--prompt-prefix', type=Path, help='Prompt prefix', default='code_generation/prompt_prefix.py')
  parser.add_argument('--prompt-suffix', type=Path, help='Prompt suffix', default='code_generation/prompt_suffix.py')
  parser.add_argument('--max-workers', type=int, help='Maximum number of workers', default=1)

  parser.add_argument('--benchmark-file', type=Path, help='Benchmark file', default='benchmark/benchmark.jsonl')

  parser.add_argument("--num-completions", type=int, default=20)
  parser.add_argument("--max-tokens", type=int, default=256)
  parser.add_argument("--top-p", type=float, default=0.95)
  parser.add_argument("--temperature", type=float, default=0.2)

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