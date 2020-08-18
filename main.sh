#!/bin/bash

pidof pigpiod >/dev/null

if [[ $? -ne 0 ]] ;
then
    sudo pigpiod
fi

python3 main.py
