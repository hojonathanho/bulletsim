#!/bin/bash
# run this on pr2base
rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro -o /tmp/pr2.urdf
rosrun collada_urdf urdf_to_collada /tmp/pr2.urdf /tmp/pr2.dae
