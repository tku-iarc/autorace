#!/usr/bin/env bash

file_dir=$(dirname "$(readlink -f "${0}")")


rule_list=$(ls "${file_dir}"/*.rules)

# Delete the created rule
for i in $rule_list; do
    rule=${i#"${file_dir}"/}
    sudo rm /etc/udev/rules.d/"${rule}"
done

# Reload usb rule
sudo udevadm control --reload-rules
udevadm trigger

echo "delete usb rule complete"
