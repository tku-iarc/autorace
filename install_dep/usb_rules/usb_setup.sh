#!/usr/bin/env bash

for i in $(dirname $0)/*.rules
do
    sudo cp ${i} /etc/udev/rules.d/
done

sudo udevadm control --reload-rules
sudo udevadm trigger
