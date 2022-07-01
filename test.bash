#!/bin/bash

declare -a maps=("ktima_march" "ktima_april" "ktima_may" "ktima_june" )
declare -a bags=("March" "April" "May" "June" )

# creating enviornmental variables  
RES_DIR=results
RES_DIR_ZIP=$RES_DIR/res_all_zip
CW_DIR=/home/ibrahim/neptune/catkin_ws
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

DELAY=10.0
T_MAX_DIFF=0.1
N_TO_ALIGN=200

#source catkin workspace
source $CW_DIR/devel/setup.bash

# create RES_DIR_ZIP -Delete (if exist)- to store the .zip files 
rm -rf $RES_DIR_ZIP
mkdir -p $RES_DIR_ZIP

# loop through all the maps and the bags 
for map in ${maps[@]}; do 
    echo '***********'
    for bag in ${bags[@]}; do
        echo "========= Map: [$map].pcd with bag: [$bag].bag ========="
        echo 'Doing ndt magic in the background'
        # the output of ndt will be saved into a rosbag called traj_[bag name]
        # roslaunch ndt_localizer ktima.launch map_id:=$map.pcd bag_filename:=$bag.bag play_delay:=$DELAY > ndt_log.txt

        MAP_BAG_DIR=$RES_DIR/$map/$bag\_bag
        MAP_BAG_ID=$map\_$bag
        rm -rf $MAP_BAG_DIR
        mkdir -p $MAP_BAG_DIR
        echo "Save results in: [$MAP_BAG_DIR]" 

        SLAM_BAG=slam_traj_$bag.bag
        NDT_BAG=traj_$bag.bag

        echo "SLAM_BAG: [$SLAM_BAG]"
        echo "NDT_BAG: [$NDT_BAG]"

        GT=gt_$MAP_BAG_ID.tum
        NDT=ndt_$MAP_BAG_ID.tum
        SLAM=slam_$MAP_BAG_ID.tum

        # save the trajectories in .tum format 
        evo_traj bag bag/$NDT_BAG /ndt_pose --save_as_tum
        evo_traj bag bag/$NDT_BAG /odometry/gps --save_as_tum
        evo_traj bag bag/$SLAM_BAG /Odometry --save_as_tum

        # move the trajectories into their dedicated folder and change their names; this is important in order to compare them using evo later! 
        mv gps.tum $MAP_BAG_DIR/$GT
        mv ndt_pose.tum $MAP_BAG_DIR/$NDT
        mv Odometry.tum $MAP_BAG_DIR/$SLAM

        https://github.com/MichaelGrupp/evo/issues/20 
        evo_ape tum $MAP_BAG_DIR/$GT \
                    $MAP_BAG_DIR/$SLAM \
                    --plot_mode xy \
                    --t_max_diff 0.01 \
                    --t_offset 0 \
                    --align_origin \
                    --n_to_align $N_TO_ALIGN \
                    --save_results $RES_DIR_ZIP/slam_$MAP_BAG_ID.zip \
                    --save_plot $MAP_BAG_DIR/slam_$MAP_BAG_ID.pdf

        evo_ape tum $MAP_BAG_DIR/$GT \
                    $MAP_BAG_DIR/$NDT \
                    --plot_mode xy \
                    --t_max_diff 0.1 \
                    --align_origin \
                    --t_offset -$DELAY \
                    --n_to_align $N_TO_ALIGN \
                    --save_results $RES_DIR_ZIP/ndt_$MAP_BAG_ID.zip \
                    --save_plot $MAP_BAG_DIR/ndt_$MAP_BAG_ID.pdf 

    done
    echo '***********'
    echo
done

