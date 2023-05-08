#!/usr/bin/env bash

wifi_file="$(dirname "$(readlink -f "${0}")")"/iwlwifi-ty-a0-gf-a0-59.ucode
sudo apt update && \
sudo apt upgrade && \
sudo cp "${wifi_file}" /lib/firmware/ && \
sudo mv /lib/firmware/iwlwifi-ty-a0-gf-a0.pnvm /lib/firmware/iwlwifi-ty-a0-gf-a0.pnvm.bak && \
sudo reboot -h
