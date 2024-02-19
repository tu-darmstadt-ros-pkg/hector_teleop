#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  add_rosrs_setup_env HECTOR_CONTROLLER_PRESET "lgitech_f710,steelseries_stratus_duo" "The  controller layout used to control the robot" 
fi
