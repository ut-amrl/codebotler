# CodeBotler Robot Interface
This utilises [ROS actions](http://wiki.ros.org/actionlib) to provide a simple interface for controlling a robot using CodeBotler. The important components are:
- `start_all.launch`: Launches all the necessary nodes
- `src/robot_actions_pkg`: Package containing the custom action messages
- `src/interface.py`: The main interface script that defines the following:
  * `go_to_client`: Action client for the "/go_to_server" action server. Associated action file is `src/robot_actions_pkg/action/GoTo.action`
  * `get_current_location_client`: Action client for the "/get_current_location_server" action server. Associated action file is `src/robot_actions_pkg/action/GetCurrentLocation.action`
  * `is_in_room_client`: Action client for the "/is_in_room_server" action server. Associated action file is `src/robot_actions_pkg/action/IsInRoom.action`
  * `say_client`: Action client for the "/say_server" action server. Associated action file is `src/robot_actions_pkg/action/Say.action`
  * `get_all_rooms_client`: Action client for the "/get_all_rooms_server" action server. Associated action file is `src/robot_actions_pkg/action/GetAllRooms.action`
  * `ask_client`: Action client for the "/ask_server" action server. Associated action file is `src/robot_actions_pkg/action/Ask.action`
  * `execute_server`: Action server for the "/execute_server" action. Associated action file is `src/robot_actions_pkg/action/Execute.action`. The `codebotler_deploy.py` script sends the generated code to this action server.

## Setup
1. Add `robot_commands` to the `ROS_PACKAGE_PATH` environment variable
1. Navigate to the `robot_interface` directory and do `catkin_make`
1. Add `source <path_to_robot_commands>/robot_interface/devel/setup.bash` to the `.bashrc` file

## Usage
The action servers catering to the actions `go_to`, `get_current_location`, `is_in_room`, `say`, `get_all_rooms`, and `ask` need to be running before the following is run.
1. Run `roslaunch robot_commands start_all.launch`