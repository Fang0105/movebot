#/bin/bash

echo "Running seeds with different random number generator seeds"
rng_seed_array=(0 100 200 300 400 500 600 700 800 900)

for i in "${rng_seed_array[@]}" ; do
    # echo "$i"
    path_file="path_$i.txt"
    ExecutionTime_file="execution_time_$i.txt"
    rng_seed=$i
    cd vamp
    ./run.sh --rng_seed $rng_seed --path_file $path_file --execution_time_file $ExecutionTime_file > /dev/null
    cd ../movebot
    ./run.sh --rng_seed $rng_seed --path_file $path_file --execution_time_file $ExecutionTime_file > /dev/null
    cd ..
done