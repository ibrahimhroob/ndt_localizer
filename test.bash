#!/bin/bash

# Declare an array of string with type
declare -a maps=("ktima_march" "ktima_april" "ktima_may" "ktima_june" )
declare -a bags=("March" "April" "May" "June" )
 
# Iterate the string array using for loop
for map in ${maps[@]}; do 
    echo '***********'
    for bag in ${bags[@]}; do
        echo ========= Map: [$map.pcd] with bag: [$bag.bag] =========
        roslaunch ndt_localizer ktima.launch map_id:=$map.pcd bag_filename:=$bag.bag > ndt_log.txt
        folder=results/$map/$bag\_bag
        file_name=$map\_$bag
        echo Save results in: $folder
        rm -rf $folder
        mkdir -p $folder
        evo_traj bag bag/traj_$bag.bag /ndt_pose --ref /odometry/gps --plot_mode xy --align_origin --n_to_align 100 -a --save_plot $folder/$file_name.pdf --t_max_diff 0.1
        evo_traj bag bag/traj_$bag.bag /ndt_pose --save_as_tum
        evo_traj bag bag/traj_$bag.bag /odometry/gps --save_as_tum
        mv *.tum $folder/
        evo_ape  tum $folder/gps.tum $folder/ndt_pose.tum --plot_mode xy --align_origin --n_to_align 100 -a --t_max_diff 0.1 --save_results $folder/$file_name.zip --save_plot $folder/ape_$file_name.pdf
    done
    echo '***********'
    echo
done

