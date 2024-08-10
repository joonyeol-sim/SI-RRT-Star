#!/bin/bash

# Environments
environments=("CircleEnv")

# Obstacles
obstacles_density=(10 20)
obstacles_density_free=(0)

# Radius
robot=(180 200)

# Time steps
timesteps=($(seq 0 49))

# Time limit in seconds (5 minutes)
time_limit=300

# Function to run a single test
run_test() {
  local env=$1
  local obs=$2
  local rad=$3
  local time=$4

  # Run the command with a timeout
  if timeout $time_limit ./build/SI-RRTStar -m "$env" -o "$obs" -r "$rad" -t "$time" -a pp; then
    echo "Test completed: -m $env -o $obs -r $rad -t $time"
  else
    echo "Test failed: -m $env -o $obs -r $rad -t $time (Timeout)"
  fi
}

export LD_LIBRARY_PATH=$HOME/CLionProjects/ompl/build/src/ompl:$LD_LIBRARY_PATH
# Loop through all combinations of parameters and run tests
for env in "${environments[@]}"; do
  if [ "$env" == "Free" ]; then
    current_obstacles_density=("${obstacles_density_free[@]}")
  else
    current_obstacles_density=("${obstacles_density[@]}")
  fi

  for obs in "${current_obstacles_density[@]}"; do
    for rad in "${robot[@]}"; do
      for time in "${timesteps[@]}"; do
        run_test "$env" "$obs" "$rad" "$time"
      done
    done
  done
done
