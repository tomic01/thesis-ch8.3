#!/bin/bash
xrandr --output HDMI1 --primary
xrandr --output HDMI1 --set audio force-dvi
xrandr --output HDMI2 --set audio force-dvi
xrandr --output HDMI1 --rotate left
xinput set-prop 'TPK USA LLC Touch Fusion 4.' 'Coordinate Transformation Matrix' 0 -1 1 1 0 0 0 0 1
exit 0
