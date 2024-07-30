#!/bin/bash

# Environments
environments=("RectEnv" "CircleEnv" "Free")

# Obstacles
obstacles=(10 20)
obstacles_free=(0 10 20)

# Radius
radius=(10 15 20 25 30 40 60 80 100)

# Time steps
timesteps=($(seq 0 49))

# Algorithm
algorithm="cbs"

# Time limit in seconds (5 minutes)
time_limit=300

# Function to run a single test
run_test() {
  local env=$1
  local obs=$2
  local rad=$3
  local time=$4

  # Run the command with a timeout
  if timeout $time_limit ./build/SI-RRTStar -m "$env" -o "$obs" -r "$rad" -t "$time" -a "$algorithm"; then
    echo "Test completed: -m $env -o $obs -r $rad -t $time -a $algorithm"
  else
    echo "Test failed: -m $env -o $obs -r $rad -t $time -a $algorithm (Timeout)"
  fi
}

# Loop through all combinations of parameters and run tests
for env in "${environments[@]}"; do
  if [ "$env" == "Free" ]; then
    current_obstacles=("${obstacles_free[@]}")
  else
    current_obstacles=("${obstacles[@]}")
  fi

  for obs in "${current_obstacles[@]}"; do
    for rad in "${radius[@]}"; do
      for time in "${timesteps[@]}"; do
        run_test "$env" "$obs" "$rad" "$time"
      done
    done
  done
done
