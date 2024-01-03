#!/bin/bash

# flash .uf2 file for mbot firmware
PWD=$(pwd)

cd $PWD/mbot_firmware
sudo ./upload.sh flash build/src/mbot.uf2
cd ..
