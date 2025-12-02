#!/bin/bash
echo 1 | sudo tee /sys/module/imx296/parameters/trigger_mode
./build/apps/rpicam-ros2 --vflip --hflip  --viewfinder-width 640 --viewfinder-height 480 -t 0 --shutter 3000 --gain 1 --awbgains 1.6,1.5 -n
