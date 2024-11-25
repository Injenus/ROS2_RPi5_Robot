#!/bin/bash

xterm -hold -e "cd src/image_processing && ./get_frames.sh" &

xterm -hold -e "cd src/image_processing && ./arrows_show.sh" &

xterm -hold -e "cd src/image_processing && ./arucos.sh" &

xterm -hold -e "cd src/pharma_delivery && ./do_by_state.sh" &

xterm -hold -e "cd src/pharma_delivery && ./state_update.sh" &

xterm -hold -e "cd src/wheel_control && ./simple_control.sh"

