# source this before running the demo
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
_DS=$HOME/disaster_sim_ws/src/disaster_sim
export GAZEBO_MODEL_PATH=$_DS/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$_DS:${GAZEBO_RESOURCE_PATH}
