#!/bin/bash
PWD=$(pwd)
#assumes you are already in mbot_ws
wget https://github.com/MBot-Project-Development/mbot_web_app/releases/download/v1.1.0/mbot_web_app-v1.1.0.tar.gz
tar -xvzf mbot_web_app-v1.1.0.tar.gz

cd mbot_web_app-v1.1.0/
./install_nginx.sh && ./install_python_deps.sh
./deploy_app.sh --no-rebuild

cd $PWD
#clean up files
rm mbot_web_app-v1.1.0.tar.gz
