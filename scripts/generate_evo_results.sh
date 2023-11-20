#!/bin/bash

# Check if the correct number of arguments are provided
if [ $# -ne 2 ]; then
  echo "Usage: $0 <ground_truth_file> <result_trajectory_file>"
  exit 1
fi

# Assign the arguments to variables
ground_truth_file=$1
result_trajectory_file=$2
# get the filename without folder and without file extension from result_trajectory_file
robot_name=$(basename $result_trajectory_file .txt)
# get the folder name from result_trajectory_file
robot_folder=$(dirname $result_trajectory_file)

# Execute evo_ape commands
ape_pose_result_file="${robot_folder}/${robot_name}_ape.zip"
# ape_trans_reuslt_file="${robot_folder}/${robot_name}_trans_ape.zip"
# ape_rot_result_file="${robot_folder}/${robot_name}_rot_ape.zip"
cmd="evo_ape tum $ground_truth_file $result_trajectory_file --align --plot --plot_mode xy --save_results $ape_pose_result_file"
echo $cmd
eval $cmd
# cmd="evo_ape tum $ground_truth_file $result_trajectory_file -r trans_part --align --plot --plot_mode xy --save_results $ape_trans_reuslt_file"
# echo $cmd
# eval $cmd
# cmd="evo_ape tum $ground_truth_file $result_trajectory_file -r rot_part --align --plot --plot_mode xy --save_results $ape_rot_result_file"
# echo $cmd
# eval $cmd

# Execute evo_rpe commands
rpe_result_file="${robot_folder}/${robot_name}_rpe.zip"
# rpe_trans_result_file="${robot_folder}/${robot_name}_trans_rpe.zip"
# rpe_rot_result_file="${robot_folder}/${robot_name}_rot_rpe.zip"
cmd="evo_rpe tum $ground_truth_file $result_trajectory_file --align --plot --plot_mode xy --save_results $rpe_result_file"
echo $cmd
eval $cmd
# cmd="evo_rpe tum $ground_truth_file $result_trajectory_file -r trans_part --align --plot --plot_mode xy --save_results $rpe_trans_result_file"
# echo $cmd
# eval $cmd
# cmd="evo_rpe tum $ground_truth_file $result_trajectory_file -r rot_part --align --plot --plot_mode xy --save_results $rpe_rot_result_file"
# echo $cmd
# eval $cmd
