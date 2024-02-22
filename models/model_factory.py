import os 
from models.AutoModel import AutoModel
from models.OpenAIChatModel import OpenAIChatModel
from models.OpenAIModel import OpenAIModel
from models.PaLMModel import PaLMModel
from models.TextGenerationModel import TextGenerationModel

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
    if args.model_type == "openai":
      model = OpenAIModel(model=args.model_name, api_key = openai_api_key)
    elif args.model_type == "openai-chat": 
      model = OpenAIChatModel(model=args.model_name, api_key=openai_api_key)
      model.set_prefix(args.chat_prompt_prefix)
    else:
      raise ValueError(f"Unknown model type: {args.model_type}")
  elif args.model_type in ["palm"]:
    # If there exists a ".palm_api_key" file, use that as the API key.
    if os.path.exists(".palm_api_key"):
      with open(".palm_api_key", "r") as f:
        palm_api_key = f.read().strip()
        print("palm_api_key", palm_api_key)
    else:
      palm_api_key = os.getenv("PALM_API_KEY")
    assert len(palm_api_key) > 0, \
        "PaLM API key not found. " + \
        "Either create a '.palm_api_key' file or " + \
        "set the PALM_API_KEY environment variable."
    if args.model_type == "palm":
      model = PaLMModel(model=args.model_name, api_key = palm_api_key)
  elif args.model_type == "automodel":
    model = AutoModel(batch_size=1, path=args.model_name)
  elif args.model_type == "hf-textgen":
    model = TextGenerationModel(args.tgi_server_url, args.max_workers)
  else:
    raise ValueError(f"Unknown model type: {args.model_type}")
  
  return model