#!/bin/bash

x_trans=(0 0.005)
y_trans=(0 0.01)
z_trans=(0 0.01)
x_rot=(0 10)
y_rot=(0 10)
z_rot=(0 10)


for xt in ${x_trans[@]}
  do for yt in ${y_trans[@]}
    do for zt in ${z_trans[@]}
	do for xr in ${x_rot[@]}
	    do for yr in ${y_rot[@]}
		do for zr in ${z_rot[@]}
		    do
			./bin/ravens --enableHaptics=0 --enableLfd=1 --enableShadows=0 --cloth=1   --friction=.1 --margin=0.005 --linkPadding=0.002 --ropeManip=0 --plotTfm=0 --useDemoLib=0 --xBias=$xt --yBias=$yt --zBias=$zt --xRot=$xr --yRot=$yr --zRot=$zr --autoLFD=1
		done
	     done
	done
      done
    done
done
