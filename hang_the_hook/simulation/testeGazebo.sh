#!/bin/bash

# Configura a Aba 1: Sim Start
echo "source ~/.bashrc; cd ~/ros2_ws/src/nectar-sdk; history -s 'make sim-start FIRMWARE=ardupilot ENV=outdoor'; READLINE_LINE=\$(history -p '!!'); READLINE_POINT=\${#READLINE_LINE}" > /tmp/.tab1
gnome-terminal --tab --title="1. Sim Start" -- bash --rcfile /tmp/.tab1 -i

# Configura a Aba 2: Sim Bridge
echo "source ~/.bashrc; cd ~/ros2_ws/src/nectar-sdk; history -s 'make sim-bridge FIRMWARE=ardupilot ENV=outdoor PROTOCOL=mavros ARGS=\"world:=/home/arthur-xavier/ros2_ws/src/black-bee-challenge-2026/simulation/world/hang_the_hook_bbc_2026.sdf\"'; READLINE_LINE=\$(history -p '!!'); READLINE_POINT=\${#READLINE_LINE}" > /tmp/.tab2
gnome-terminal --tab --title="2. Sim Bridge" -- bash --rcfile /tmp/.tab2 -i

# Configura a Aba 3: ROS 2 Node
echo "source ~/.bashrc; cd ~/ros2_ws && source install/setup.bash; history -s 'ros2 run hang_the_hook mangalarga'; READLINE_LINE=\$(history -p '!!'); READLINE_POINT=\${#READLINE_LINE}" > /tmp/.tab3
gnome-terminal --tab --title="3. ROS 2 Node" -- bash --rcfile /tmp/.tab3 -i

# Configura a Aba 4: Sim Stop (Nova aba para encerrar)
echo "source ~/.bashrc; cd ~/ros2_ws/src/nectar-sdk; history -s 'make sim-stop'; READLINE_LINE=\$(history -p '!!'); READLINE_POINT=\${#READLINE_LINE}" > /tmp/.tab4
gnome-terminal --tab --title="4. Sim Stop" -- bash --rcfile /tmp/.tab4 -i
