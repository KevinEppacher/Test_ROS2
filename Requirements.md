# Requirements

sudo apt update
sudo apt install python3 python3-pip
pip install casadi
pip install numpy scipy matplotlib


# Enter in each Terminal

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevin/Documents/Master/2_Semester/Spez/walle_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


pkill -9 gazebo
pkill -9 gzserver
pkill -9 gzclient
sudo lsof -i :11345
pkill -9 gazebo && pkill -9 gzserver && pkill -9 gzclient




