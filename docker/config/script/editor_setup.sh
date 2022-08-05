#!/usr/bin/env bash

for i in ./editor/*rc; do
    cp ./${i} /home/${USER}/.${i}
done
