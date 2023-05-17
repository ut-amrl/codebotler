import os
import openai
import sys
import asyncio
import json
import websockets
import time
import argparse
import signal
import rospy
from std_msgs.msg import String

# If there exists a ".openai_api_key" file, use that as the API key.
if os.path.exists(".openai_api_key"):
    with open(".openai_api_key", "r") as f:
        openai.api_key = f.read().strip()
else:
    openai.api_key = os.getenv("OPENAI_API_KEY")

with open("gpt_prompt.md", "r") as f:
    pre_prompt = f.read()


async def get_code(websocket, data):
    start_time = time.time()
    prompt = pre_prompt + f" \"{data['text']}\"" + "\nProgram:\n"
    openai_response = openai.Completion.create(
        model="text-davinci-003",
        prompt=prompt,
        temperature=0.7,
        max_tokens=256,
    )
    end_time = time.time()
    # Print the response time with 2 decimal places.
    print(f"Response time: {round(end_time - start_time, 2)} seconds")
    code = openai_response.choices[0].text
    # Strip "```python" and "```" from the code
    code = code.replace("```python", "")
    code = code.replace("```", "")
    # Remove any leading or trailing end-of-line characters
    code = code.strip()
    pub.publish(code)
    response = {"code": f"{code}"}

    # Convert the response to JSON and send it back to the client
    await websocket.send(json.dumps(response))


async def eval(websocket, data):
    print("Received eval request:")
    print(data)
    # Open the file named eval.json and append the data to it.
    with open("eval.json", "a") as f:
        f.write(json.dumps(data) + "\n")
    await websocket.send(json.dumps({}))


async def handle_message(websocket, message):
    # Parse incoming message as JSON
    data = json.loads(message)
    if data['type'] == 'code':  # basically the html client sends the input prompt to the server with the type "code", i.e., when you press enter
        print("Received code request")
        await get_code(websocket, data)  # processes the prompt using model and sends back the code to client
    elif data['type'] == 'eval':  # when html client sends the task to be stored, i.e., when a eval button is pressed
        await eval(websocket, data)
    else:
        print("Unknown message type: " + data['type'])


async def ws_main(websocket, path):
    print("INFO: A client connected")
    try:
        async for message in websocket:
            await handle_message(websocket, message)
    except websockets.exceptions.ConnectionClosed:
        print("INFO: A client disconnected")


def main(args):
    # Create an asyncio event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(ws_main, args.ip, args.port)

    # Register a signal handler to stop the server when Ctrl-C is pressed
    loop.add_signal_handler(signal.SIGINT, lambda: (print("INFO: Shutting down Server"), loop.stop()))

    try:
        server = loop.run_until_complete(start_server)
        loop.run_forever()
    except Exception as e:
        print("ERROR_INFO: " + str(e))
    finally:
        print("Closing server")
        for task in asyncio.all_tasks(loop=loop):
            task.cancel()
        server.close()
        loop.run_until_complete(server.wait_closed())
        loop.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # Add the desired command line arguments
    parser.add_argument('--ip', type=str, help='IP address', default="localhost")
    parser.add_argument('--port', type=int, help='Port number', default=8190)

    # Parse the command line arguments
    args = parser.parse_args()

    rospy.init_node('python_commands_publisher')
    pub = rospy.Publisher('/python_commands', String, queue_size=1)
    main(args)
