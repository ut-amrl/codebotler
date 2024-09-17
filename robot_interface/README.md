# CodeBotler Robot Interface

CodeBotler utilizes [ROS actions](http://wiki.ros.org/actionlib) to execute generated code on a real robot.
[robot_interface/src/robot_client_interface.py](src/robot_client_interface.py) defines the action clients that CodeBotler uses to call the robot action servers. The action definitions are in [robot_interface/src/robot_actions_pkg/action](src/robot_actions_pkg/action). An example robot action server script is provided in [robot_interface/src/robot_server_example.py](src/robot_server_example.py) - the example server simply prints action calls to the terminal, along with the call arguments and the result.
You can use this example script as a template to implement the action servers for your robot.

## Build
To build the action messages and the CodeBotler client:
- navigate to the `robot_interface` subdirectory and run `./setup_robot.sh`
- add `devel/setup.bash` source command to your `.bashrc` file (just after the source command for your ROS distribution)

## Usage
The robot-specific action server must be launched before running the deployment interface.
1. Launch your robot action server. For example, for the spot amrl server:
    ```bash
    ./codebotler_amrl_impl/start_all.sh  # basically roslaunch codebotler_amrl_impl start_all.launch
    ```
1. Launch the deployment script on the robot:
    ```bash
    python3 codebotler.py --robot --ip <robot_ip>   # e.g., for spot: python3 codebotler.py --robot --ip 10.1.0.3
    ```
1. Open `http://<robot_ip>:8080/` in your browser to access the deployment interface.
