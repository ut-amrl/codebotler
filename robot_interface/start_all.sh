#!/bin/bash

ut_jackal_path=$(realpath third_party/ut_jackal)
echo $ut_jackal_path
graph_nav_path=$(realpath third_party/ut_jackal/graph_navigation)
echo $graph_nav_path

if [[ $ROS_PACKAGE_PATH == *"ut_jackal"* ]]; then
    echo "Removing ut_jackal from ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | tr ':' '\n' | grep -v "ut_jackal" | paste -sd: -)
fi
# Add the new path to ROS_PACKAGE_PATH
if [[ $ROS_PACKAGE_PATH != *"$ut_jackal_path"* ]]; then
    echo "Adding $ut_jackal_path to ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$ut_jackal_path:$ROS_PACKAGE_PATH
fi

if [[ $ROS_PACKAGE_PATH == *"graph_navigation"* ]]; then
    echo "Removing graph_navigation from ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | tr ':' '\n' | grep -v "graph_navigation" | paste -sd: -)
fi
# Add the new path to ROS_PACKAGE_PATH
if [[ $ROS_PACKAGE_PATH != *"$graph_nav_path"* ]]; then
    echo "Adding $graph_nav_path to ROS_PACKAGE_PATH..."
    export ROS_PACKAGE_PATH=$graph_nav_path:$ROS_PACKAGE_PATH
fi

roslaunch robot_commands start_all.launch
