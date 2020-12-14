#!/bin/bash

#uruchom gazebo, a nastepnie zalezne node'y
#pliki launch rosa nie obsługują delaya miedzy ladowaniami node'ow, co generuje bledy

roslaunch omnivelma_nav_setup omnivelma.launch &
PIDGAZ=$!
sleep 5
# read -n 1 -p "Press any button to continue loading..." -s

trap "kill $PIDGAZ" SIGINT
echo "Wciśnij ^C, aby wyłączyć procesy"
sleep infinity
