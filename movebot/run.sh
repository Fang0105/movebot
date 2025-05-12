#!/bin/bash



input_dir="input"
output_dir="output"

arm_description_file="arm_description.txt"
start_end_file="start_end.txt"
path_file="path.txt"

max_iterations=1000
reach_threshold=2

set -x

./movebot ${input_dir}/${arm_description_file} ${input_dir}/${start_end_file} ${output_dir}/${path_file} ${max_iterations} ${reach_threshold}