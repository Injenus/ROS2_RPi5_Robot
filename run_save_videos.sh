#!/bin/bash

xterm -hold -e "cd src/image_processing && ./get_frames.sh" &

xterm -hold -e "cd src/image_processing && ./save_videos.sh"

