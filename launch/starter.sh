#!/bin/bash


trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    sudo kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}

sudo LED_server &
sleep 2
ROS_NAMESPACE="$HOSTNAME" rosrun rpi_led_consensus node.py &

cat
