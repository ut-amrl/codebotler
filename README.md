
## Running the Web interface:

1. Create a file named `.openai_api_key` with your OpenAI API key in it.
2. Run `python interface-server.py` to start the server.
3. Open the file `interface.html` in your browser.
4. Enter a task description in the text box and hit enter.
5. Once the code is generated, click on one of the buttons to indicate your
   assesment of the code's quality.
6. Repeat steps 4 and 5 for as many tasks as you want.
7. The evaluations will be saved in `eval.json`.

Example result in `eval.json`:
```json
{"type": "eval", "task": "Which room is the dog in?", "code": "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\ndog_found = False\ndog_loc = None\nfor room in list_of_rooms:\n    go_to(room)\n    if is_in_room(\"dog\"):\n        dog_found = True\n        dog_loc = room\n        break\ngo_to(start_loc)\nif dog_found:\n    say(\"The dog is in the \" + dog_loc)\nelse:\n    say(\"The dog is not in any room\")", "result": "Correct"}
{"type": "eval", "task": "Find a conference room with a projector.", "code": "list_of_rooms = get_all_rooms()\nstart_loc = get_current_location()\nconf_room_found = False\nconf_room_loc = None\nfor room in list_of_rooms:\n    go_to(room)\n    if is_in_room(\"projector\"):\n        conf_room_found = True\n        conf_room_loc = room\n        break\ngo_to(start_loc)\nif conf_room_found:\n    say(\"There is a conference room with a projector in the \" + conf_room_loc)\nelse:\n    say(\"There is no conference room with a projector in the house\")", "result": "Correct"}
```

Important files/folders:
 - `train/` : training data (OUTDATED)
 - `test/` : testing data (OUTDATED)
 - `DSL.txt` : the prompt, which contains the DSL and a couple of examples
 - `train.py` : finetuning the model, based off `https://github.com/loubnabnl/santacoder-finetuning`
   - This currently isn't being used, feeding prompts to chatGPT for now

 Files relating to code "interpreter" are in `/robot_interpreter/`
 - `code.txt` : the code to be interpreted, in format specified by `DSL.txt`
 - `write_code.py` : the actual interpreter, converts code in `code.txt` to python
 - `robot_lib.py` : where the interpreter saves the generated code in a runnable format
 - `robot_lib_template.txt` : template for interpreting code, used by `write_code.py`
 - Usage: put the code you want to run in `code.txt`, then run `write_code.py`
