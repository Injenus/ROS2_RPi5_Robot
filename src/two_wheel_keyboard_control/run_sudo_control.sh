#!/bin/bash

xterm -hold -e "./run_keyboard_talker.sh" &

xterm -hold -e "./run_com_listener.sh"

#xterm -hold -e "./run_com_listener_foo.sh"