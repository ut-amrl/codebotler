# CodeBotler Robot Interface

CodeBotler utilizes [ROS2 actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) to execute generated code on a real robot.
[robot_interface/src/robot_client_interface.py](src/robot_client_interface.py) defines the action clients that CodeBotler uses to call the robot action servers. The action definitions are in [robot_interface/src/robot_actions_pkg/action](src/robot_actions_pkg/action). An example robot action server script is provided in [robot_interface/src/robot_server_example.py](src/robot_server_example.py) - the example server simply prints action calls to the terminal, along with the call arguments and the result.
You can use this example script as a template to implement the action servers for your robot.

## Build
To build the action messages and the CodeBotler client:
1. Navigate to the `robot_interface` subdirectory and run `colcon build`:
    ```bash
    cd robot_interface
    colcon build
    ```
1. Source the ROS2 workspace setup script:
    ```bash
    source install/setup.bash
    ```
1. Optionally, add the ROS2 workspace setup script to your `~/.bashrc` file to automatically source the setup script when opening a new terminal.
    ```bash
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
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