import os
import openai
import sys
import asyncio
import json
import websockets
import time

# If there exists a ".openai_api_key" file, use that as the API key.
if os.path.exists(".openai_api_key"):
    with open(".openai_api_key", "r") as f:
        openai.api_key = f.read().strip()
else:
  openai.api_key = os.getenv("OPENAI_API_KEY")

with open("chatgpt_prompt.md", "r") as f:
    pre_prompt = f.read()

async def handle_message(websocket, message):
    # Parse incoming message as JSON
    data = json.loads(message)
    # print(f"Received: {data}")
    # Time the response.
    start_time = time.time()
    prompt = pre_prompt + f" \"{data['text']}\"" + "\nProgram:\n"
    openai_response = openai.Completion.create(
      model="text-davinci-003",
      prompt=prompt,
      temperature=0.7,
      max_tokens=256,
      top_p=1,
      frequency_penalty=0,
      presence_penalty=0
    )
    end_time = time.time()
    # Print the response time with 2 decimal places.
    print(f"Response time: {round(end_time - start_time, 2)} seconds")
    code = openai_response.choices[0].text
    # Strip "```python" and "```" from the code
    code = code.replace("```python", "")
    code = code.replace("```", "")
    response = {"code": f"{code}"}
    
    # Convert the response to JSON and send it back to the client
    await websocket.send(json.dumps(response))

async def ws_main(websocket, path):
    async for message in websocket:
        await handle_message(websocket, message)
        
def main(argv):
    # Create an asyncio event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(ws_main, "localhost", 8190)

    loop.run_until_complete(start_server)
    loop.run_forever()

if __name__ == "__main__":
    main(sys.argv[1:])
