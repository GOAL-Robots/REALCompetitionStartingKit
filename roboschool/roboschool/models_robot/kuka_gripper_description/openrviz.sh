#!/bin/bash

deactivate
source ../../devel/setup.sh
roslaunch kuka_gripper_description display.launch model:=urdf/kuka_gripper.urdf

