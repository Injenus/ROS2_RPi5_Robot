#!/bin/bash

xterm -hold -e "cd /src/two_wheel_keyboard_control && ./run_keyboard_talker.sh" &

xterm -hold -e "cd /src/two_wheel_keyboard_control && ./run_com_listener.sh"
