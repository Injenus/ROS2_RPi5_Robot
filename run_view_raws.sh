#!/bin/bash

xterm -hold -e "cd /src/image_processing && ./get_frames.sh" &

xterm -hold -e "cd /src/image_processing && ./view_raws.sh"

