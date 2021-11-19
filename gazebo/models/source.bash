DIR=$(dirname $(realpath "${BASH_SOURCE[0]}"))

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$DIR
