import os 
from roboeval.models.OpenAIChatModel import OpenAIChatModel
from roboeval.models.GeminiModel import GeminiModel
from vllm import LLM

def load_model(args):
  if "openai" in args.model_type:
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
    llm = OpenAIChatModel(model=args.model_name_or_path, api_key=openai_api_key)    
  elif "gemini" in args.model_type:
    # If there exists a ".gemini_api_key" file, use that as the API key.
    if os.path.exists(".gemini_api_key"):
      with open(".gemini_api_key", "r") as f:
        gemini_api_key = f.read().strip()
    else:
      gemini_api_key = os.getenv("GEMINI_API_KEY")
    assert len(gemini_api_key) > 0, \
        "Gemini API key not found. " + \
        "Either create a '.gemini_api_key' file or " + \
        "set the GEMINI_API_KEY environment variable."
    llm = GeminiModel(model=args.model_name_or_path, api_key=gemini_api_key)
  elif "vllm" in args.model_type:
    llm = LLM(model=args.model_name_or_path,
              tensor_parallel_size=args.tensor_parallel_size,
              gpu_memory_utilization=args.gpu_memory_utilization)
  else:
    raise ValueError(f"To Be Implemented: {args.model_type}")
  
  return llm