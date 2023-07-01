# CodeBotler Robot Interface
The robot interface utilizes [ROS actions](http://wiki.ros.org/actionlib) to execute CodeBotler robot actions. The important components are:
- `robot_interface/src/robot_actions_pkg`: ROS Catkin Package containing the custom action messages
- `robot_interface/src/interface.py`: The main interface script that defines the CodeBotler robot actions:
  * `go_to_client`: Action client for the `/go_to_server` action server. The associated action file is `src/robot_actions_pkg/action/GoTo.action`
  * `get_current_location_client`: Action client for the `/get_current_location_server` action server. The associated action file is `src/robot_actions_pkg/action/GetCurrentLocation.action`
  * `is_in_room_client`: Action client for the `/is_in_room_server` action server. The associated action file is `src/robot_actions_pkg/action/IsInRoom.action`
  * `say_client`: Action client for the `/say_server` action server. The associated action file is `src/robot_actions_pkg/action/Say.action`
  * `get_all_rooms_client`: Action client for the `/get_all_rooms_server` action server. The associated action file is `src/robot_actions_pkg/action/GetAllRooms.action`
  * `ask_client`: Action client for the `/ask_server` action server. The associated action file is `src/robot_actions_pkg/action/Ask.action`
- `codebotler_deploy.py`: This starts up the HTTP server serving the `interface.html` interface to type in natural language commands, as well as the WebSocket server that internally calls `robot_interface/src/interface.py` to execute the generated code


## Setup
1. Navigate to the `robot_interface` directory and do `catkin_make`
1. Add `source <path_to_robot_commands>/robot_interface/devel/setup.bash` line to the `.bashrc` file

## Usage
The action servers for the CodeBotler actions must be launched on the robot before running the deployment interface. An example robot action server script is provided [src/robot_example.py](src/robot_example.py). 
1. Run `python codebotler_deploy.py --ip <robot_ip> --robot` on the robot
1. Open `http://<robot_ip>:8080/` in your browser and type in natural language tasks
