#!/bin/bash

CW_PTH=/mnt/2564a3b0-8e42-43a4-b917-2b1af0c78052/catkin_ws
ODOM_TOPIC=/Odometry

source $CW_PTH/devel/setup.bash

declare -a bags=("March" "April" "May" "June" )

for bag in ${bags[@]}; do
    echo "========= Creating 3D map from [$bag].bag ========="
    roslaunch fast_lio mapping_ktima_from_bag.launch bag_filename:=$bag.bag scan_name:=$bag play_rate:=4 &> fastlio_log.txt
    echo ""
done

echo "Done!"
