#!/usr/bin/env bash

file_dir=$(dirname "$(readlink -f "${0}")")

# Copy the created USB rule
sudo cp "${file_dir}"/*.rules /etc/udev/rules.d/

# Reload usb rule
sudo udevadm control --reload-rules && \
udevadm trigger

echo "load usb rule complete"
