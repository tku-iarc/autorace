#!/usr/bin/env bash

sudo apt update && \
sudo apt upgrade -y && \
sudo apt install -y --no-install-recommends \
    python3-pip && \
sudo -H pip3 install -U jetson-stats && \
sudo systemctl restart jetson_stats.service && \
sudo nvpmodel -m 0 && \
sudo reboot

# pwm fan
# jtop

# sudo sh -c 'echo X > /sys/class/thermal/cooling_deviceY/cur_state'
# sudo jetson_clocks
