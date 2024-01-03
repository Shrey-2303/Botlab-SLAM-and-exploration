#!/bin/bash
set -e  # Quit on error.

# Python installation.
echo "Installing the Python MBot Bridge code..."
echo
sudo pip install .

# Websockets C++ dependency installation.
echo
echo "Installing dependencies for the API..."
echo
wget https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.tar.gz
tar -xzf 0.8.2.tar.gz
cd websocketpp-0.8.2/
mkdir build && cd build
cmake ..
make
sudo make install

echo
echo "Cleaning up..."
echo
cd ../../
rm 0.8.2.tar.gz
rm -rf websocketpp-0.8.2/

# C++ API installation.
echo
echo "Building the C++ MBot API..."
echo
python3 setup.py build_ext

echo
echo "Installing the C++ MBot API..."
echo
sudo python3 setup.py install_ext

# Install service.
echo
echo "Installing the MBot Bridge service..."
echo
# Copy the services.
sudo cp services/mbot-bridge.service /etc/systemd/system/mbot-bridge.service

# Enable the services.
sudo systemctl daemon-reload
sudo systemctl enable mbot-bridge.service
sudo systemctl restart mbot-bridge.service

echo
echo "MBot Bridge setup complete."
echo
