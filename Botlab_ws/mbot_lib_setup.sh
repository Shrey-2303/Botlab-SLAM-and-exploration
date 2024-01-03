#!/bin/bash

PWD=$(pwd)

cd rplidar_lcm_driver/
./scripts/install.sh

cd $PWD/mbot_autonomy/
./scripts/install.sh

cd $PWD/mbot_bridge/
./install_scripts.sh