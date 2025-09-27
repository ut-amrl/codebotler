# CodeBotler Robot Interface

CodeBotler utilizes [ROS2 actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) to execute generated code on a real robot.
[robot_interface/src/robot_client_interface.py](src/robot_client_interface.py) defines the action clients that CodeBotler uses to call the robot action servers. The action definitions are provided by the `cobot_codebotler_actions` package. An example robot action server script is provided in [robot_interface/src/robot_server_example.py](src/robot_server_example.py) - the example server simply prints action calls to the terminal, along with the call arguments and the result.
You can use this example script as a template to implement the action servers for your robot.

## Setup
To set up the CodeBotler robot interface:
1. Ensure the `cobot_codebotler_actions` package is built and sourced in your ROS2 environment
2. Install Python dependencies:
    ```bash
    cd robot_interface
    pip install -r requirements.txt
    ```

## Usage
The robot-specific action server must be launched before running the deployment interface.
1. Launch your robot action server. To launch the example server:
    ```bash
    python3 robot_interface/src/robot_server_example.py
    ```
1. Launch the codebotler script on the robot:
    ```bash
    python3 codebotler.py --robot --ip <robot_ip>
    ```
1. Open `http://<robot_ip>:8080/` in your browser to access the deployment interface.