#!/bin/bash

# This script is used to automatically ssh to mbot server.
# It is used to avoid typing IP address and password every time.

# password: -p i<3robots!

# ip varies
IP_ADDR="67.194.14.225"
HOSTNAME="mbot"
PWD="i<3robots!"

# ssh to mbot
# ssh $HOSTNAME@$IP_ADDR

sshpass -p $PWD ssh $HOSTNAME@$IP_ADDR