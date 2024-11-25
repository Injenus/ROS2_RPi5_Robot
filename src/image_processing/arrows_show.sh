#!/bin/bash
bash -c "source /home/inj/ros2_iron/install/setup.bash  && source /home/inj/ros2_iron/src/image_processing/.venv/bin/activate && ros2 run image_processing arrows --ros-args -p is_show:=1""