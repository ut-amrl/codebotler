#!/usr/bin/env python3

robot_interface_available = False
eval_model = None
try:
    from utilities import *
    add_pythonpath_load_amrl_msgs_cd_rel(".", ".")
    import rospy
    from std_msgs.msg import String
    robot_interface_available = True
except:
    print("Could not import rospy. Robot interface is not available.")
    pass

import os
import openai
import sys
import asyncio
import json
import websockets
import time
import argparse
import signal
import starcoder

# If there exists a ".openai_api_key" file, use that as the API key.
if os.path.exists(".openai_api_key"):
    with open(".openai_api_key", "r") as f:
        openai.api_key = f.read().strip()
else:
    openai.api_key = os.getenv("OPENAI_API_KEY")

with open("gpt_prompt.md", "r") as f:
    pre_prompt = f.read()


async def get_code(websocket, data):
    global robot_interface_available
    global eval_model
    start_time = time.time()
    prompt = pre_prompt + f" \"{data['text']}\"" + "\nProgram:\n```python\n"
    if eval_model == "davinci":
        openai_response = openai.Completion.create(
            model="text-davinci-003",
            # model="gpt-3.5-turbo",
            # model="code-davinci-002",
            prompt=prompt,
            temperature=0.7,
            max_tokens=512,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0,
            stop="```"
        )
        code = openai_response.choices[0].text
    elif "starcoder" in eval_model:
        words = eval_model.split("_")
        code = starcoder.get_code_string(
            ip=words[1],
            port=words[2],
            prompt=prompt,
            temperature=0.1,
            max_tokens=512,
            top_p=0.7,
            repetition_penalty=None,
            stop="```"
        )
    else:
        raise ValueError("Unknown model: " + eval_model)
    end_time = time.time()
    # Print the response time with 2 decimal places.
    print(f"Response time: {round(end_time - start_time, 2)} seconds")
    # Strip "```python" and "```" from the code
    code = code.replace("```python", "")
    code = code.replace("```", "")
    # Remove any leading or trailing end-of-line characters
    code = code.strip()
    if robot_interface_available:
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
    global eval_model
    # Create an asyncio event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(ws_main, args.ip, args.port)
    eval_model = args.model

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
    parser.add_argument('--model', type=str, help='Model name: davinci/starcoder_sip_sport', default='davinci')

    # Parse the command line arguments
    if robot_interface_available:
        args = parser.parse_args(rospy.myargv()[1:])
    else:
        args = parser.parse_args(sys.argv[1:])

    if robot_interface_available:
        rospy.init_node('python_commands_publisher')
        pub = rospy.Publisher('/chat_commands', String, queue_size=1)
    main(args)
