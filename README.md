
## Running the Code Generator Web interface:

1. Navigate to the `code_generator` subdirectory.
2. Create a file named `.openai_api_key` with your OpenAI API key in it.
3. To start the server, run the following:
```
python interface_server.py
```
It has the following optional flags:
   - `--ip`: The IP address to host the server to (default is `localhost`).
   - `--port`: The port to host the server to (default is `8190`).
   - `--model`: It is either text-davinci-003 (default) or starcoder:
      - to specify the text-davinci-003 model, use `--model davinci`
      - to specify the starcoder model, use `--model starcoder_sip_sport` where `sip` and `sport` are the IP address and port of the starcoder server.
4. Modify `interface.html` (if need be) to make sure to use the same `ip` and `port` as in step 3. Open the file `interface.html` in your browser.
5. Enter a task description in the text box and hit enter.
6. Once the code is generated, click on one of the buttons to indicate your
   assesment of the code's quality.
6. Repeat steps 4 and 5 for as many tasks as you want.
7. The evaluations will be saved in `eval.json`.

Example result in `eval.json`:
```json
{"type": "eval", "task": "Which room is the dog in?", "code": "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\ndog_found = False\ndog_loc = None\nfor room in list_of_rooms:\n    go_to(room)\n    if is_in_room(\"dog\"):\n        dog_found = True\n        dog_loc = room\n        break\ngo_to(start_loc)\nif dog_found:\n    say(\"The dog is in the \" + dog_loc)\nelse:\n    say(\"The dog is not in any room\")", "result": "Correct"}
{"type": "eval", "task": "Find a conference room with a projector.", "code": "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\nconf_room_found = False\nconf_room_loc = None\nfor room in list_of_rooms:\n    go_to(room)\n    if is_in_room(\"projector\"):\n        conf_room_found = True\n        conf_room_loc = room\n        break\ngo_to(start_loc)\nif conf_room_found:\n    say(\"There is a conference room with a projector in the \" + conf_room_loc)\nelse:\n    say(\"There is no conference room with a projector in the house\")", "result": "Correct"}
```
