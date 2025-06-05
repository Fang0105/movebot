#!/bin/bash



input_dir="input"
output_dir="output"

arm_description_file="arm_description.txt"
start_end_file="start_end.txt"
path_file="path.txt"
# if [ -z "$1" ]; then
#   path_file="path.txt"
#   echo "No path file specified, using default: path.txt"
# else
#   path_file=$1
# fi

ExecutionTime_file="execution_time.txt"
# if [ -z "$2" ]; then
#   ExecutionTime_file="execution_time.txt"
#   echo "No execution time file specified, using default: execution_time.txt"
# else
#   ExecutionTime_file=$2
# fi

obstacles_file="obstacles.txt"

max_iterations=10000
reach_threshold=2
rng_seed=100
# if [ -z "$3" ]; then
#   rng_seed=100
# else
#   rng_seed=$3
# fi

# 開始解析參數
while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--path_file)
      path_file="$2"
      shift 2
      ;;
    --path_file=*)
      path_file="${1#*=}"
      shift
      ;;
    -e|--execution_time_file)
      ExecutionTime_file="$2"
      shift 2
      ;;
    --execution_time_file=*)
      ExecutionTime_file="${1#*=}"
      shift
      ;;
    -r|--rng_seed)
      rng_seed="$2"
      shift 2
      ;;
    --rng_seed=*)
      rng_seed="${1#*=}"
      shift
      ;;
    --help)
      show_help
      exit 0
      ;;
    -*)
      echo "未知的參數: $1" >&2
      show_help
      exit 1
      ;;
    *)
      echo "忽略未識別參數: $1"
      shift
      ;;
  esac
done

set -x

./vamp ${input_dir}/${arm_description_file} ${input_dir}/${start_end_file} ${output_dir}/${path_file} ${input_dir}/${obstacles_file} ${output_dir}/${ExecutionTime_file} ${max_iterations} ${reach_threshold} ${rng_seed}