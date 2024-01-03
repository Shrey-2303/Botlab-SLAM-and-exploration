#!/bin/bash

# This script automatically builds changes from mbot_firmware
# and uploads them to mbot server.

#set of commands to build
cd mbot_firmware/build
cmake ..
make

# flash .uf2 file for mbot firmware
cd ..
sudo ./upload.sh flash build/src/mbot.uf2

cd ..
