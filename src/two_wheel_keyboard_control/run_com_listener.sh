#!/bin/bash
sudo -E bash -c "source /home/inj/ros2_iron/install/setup.bash && source /home/inj/ros2_iron/src/two_wheel_keyboard_control/.venv/bin/activate && ros2 run two_wheel_keyboard_control com_listener"
