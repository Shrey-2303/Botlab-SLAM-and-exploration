#!/bin/bash

#build firmware for any changes
source build_script.sh
PWD=$(pwd)

# flash calibration .uf2 file for mbot calibration
cd mbot_firmware
sudo ./upload.sh flash build/tests/mbot_calibrate_classic.uf2

sleep 60
# flash mbot .uf2 file to update firmware
sudo ./upload.sh flash build/src/mbot.uf2

cd ..
