#!/bin/bash 

git submodule update --init --recursive

ut_jackal_path=$(realpath third_party/ut_jackal)
echo $ut_jackal_path
graph_nav_path=$(realpath third_party/ut_jackal/graph_navigation)
echo $graph_nav_path

cd third_party/GroundingDINO
pip install -q -e .

mkdir weights/
cd weights/
if test -f groundingdino_swint_ogc.pth; then
  echo "model weights already exists, skipping installation"
else
  echo "downloading model weights"
  wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
fi
cd ../

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

cd ../ut_jackal
make -j$(nproc)
cd ../../

# Give execute permissions to all scripts
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
cd ../code_generator
find . -maxdepth 1 \( -name "*.py" -o -name "*.sh" \) -exec chmod +x {} \;
