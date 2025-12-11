#!/usr/bin/env python3

import asyncio
import websockets
import json
import argparse
import sys

async def send_request(uri, prompt, execute=False, verbose=True):
    try:
        async with websockets.connect(uri) as websocket:
            request = {
                "type": "code",
                "prompt": prompt,
                "execute": False 
            }
            if verbose:
                print(f"Sending request: {json.dumps(request, indent=2)}")
            await websocket.send(json.dumps(request))
            
            response_message = await websocket.recv()
            response = json.loads(response_message)
            
            if verbose:
                print(f"Raw Response: {json.dumps(response, indent=2)}")
                if "code" in response:
                    print(f"\nGenerated Code:")
                    print("--------------------------------------------------")
                    print(response["code"])
                    print("--------------------------------------------------")
                    print(f"Timing: {response.get('timing', 'N/A')} seconds")
                else:
                    print(f"Received non-code response: {response}")
            
            # If execute is True and we got code, send the execute request
            if execute and "code" in response:
                execute_request = {
                    "type": "execute",
                    "task": prompt,
                    "code": response["code"]
                }
                if verbose:
                    print(f"Sending execute request: {json.dumps(execute_request, indent=2)}")
                
                await websocket.send(json.dumps(execute_request))
                
                # We might not get a response for execute, or maybe we do?
                # The server code 'execute' function prints but doesn't seem to send a response back on the websocket immediately 
                # unless post_transcript/post_code happens.
                # But handle_message doesn't await anything after execute().
                # Codebotler server:
                # elif data['type'] == 'execute':
                #   execute(data['code'])
                
                # So we probably won't receive a specific confirmation message for 'execute' type from handle_message.
                # But let's assume we proceed.

            return response

    except ConnectionRefusedError:
        error_msg = f"Error: Could not connect to server at {uri}. Is Codebotler running?"
        if verbose:
            print(error_msg)
        return {"error": error_msg}
    except Exception as e:
        error_msg = f"An error occurred: {e}"
        if verbose:
            print(error_msg)
        return {"error": error_msg}

def main():
    parser = argparse.ArgumentParser(description="CLI for Codebotler Code Generation")
    parser.add_argument("prompt", type=str, help="Natural language prompt for code generation")
    parser.add_argument("--host", type=str, default="localhost", help="Server host (default: localhost)")
    parser.add_argument("--port", type=int, default=8190, help="Server port (default: 8190)")
    parser.add_argument("--execute", action="store_true", help="Execute the generated code on the server")

    args = parser.parse_args()
    
    uri = f"ws://{args.host}:{args.port}"
    
    asyncio.run(send_request(uri, args.prompt, args.execute))

if __name__ == "__main__":
    main()
