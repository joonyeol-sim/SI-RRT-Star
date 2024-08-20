# SI-RRT*

## Overview

This repository includes a two-level approach for Multi-Robot Path Planning (MRPP) in continuous space, aiming to find conflict-free paths for individual robots while efficiently handling inter-robot conflicts at a higher level. 
At the low level, **SI-RRT*** employs a sampling-based planner that utilizes safe time intervals to navigate in continuous space without the need for discrete time steps, enabling the planning of collision-free trajectories for individual robots with fewer samples. 
At the high level, it can integrate with any conflict resolution method, with examples being Safe Interval Continuous space Prioritized Planning (SI-CPP) for improved scalability and Safe Interval Continuous space Conflict-Based Search (SI-CCBS) for higher-quality solutions.

## Source Code
### Requirement
```bash
sudo apt-get install g++ cmake libyaml-cpp-dev python3-matplotlib
```


### Build
```bash
./compile.sh
```
### Benchmark File Structure
- The benchmark files are located in the `benchmarks` folder.
    - The `benchmarks` folder contains three subfolders: `CircleEnv_10`, `CircleEnv_20`, `Free_0`, `RectEnv_10`, and `RectEnv_20`.
    - Each subfolder contains the number of agents as a subfolder (e.g., `10`, `15`, `20`, `25`, `30`, `40`, `60`, `80`, `100`, `120`, `140`, `160`, `180`, `200`).
    - Each agent subfolder contains 50 test instances.
- The solution folder and data folder have the same structure as the benchmark folder.
  - **To extract the solution and data, you must create a solution folder with the same structure as the benchmark.**
  - e.g., `solution/CircleEnv_10/agents60/` is the solution folder for the `CircleEnv` with 10 obstacle density and 60 agents.
  - e.g., `data/CircleEnv_10/agents60/` is the data folder for the `CircleEnv` with 10 obstacle density and 60 agents.

### Run example instances
```bash
# make sure your are in build folder
# -m is the name of the environment (Free, CircleEnv, RectEnv)
# -o is the obstacle density of the environment (0, 10, 20) (0 for Free) (10, 20 for RectEnv)
# -r is the number of agents (10, 15, 20, 25, 30, 40, 60, 80, 100)
# -t is the test number ([0-49])
# -a is the algorithm (pp, cbs)

# This command will run the SI-CPP algorithm on the CircleEnv with 10 obstacle density and 60 agents for the 0th instance.
./build/SI-RRTStar -m CircleEnv -o 10 -r 60 -t 0 -a pp
```

### Visualize Results
We provide a python script to visualize the results.
The agent is represented as a circle with the radius of the agent and moves along the path.
```bash
# make sure your are in build folder
# -m is the name of the environment (Free, CircleEnv, RectEnv)
# -o is the obstacle density of the environment (0, 10, 20) (0 for Free) (10, 20 for RectEnv)
# -r is the number of agents (10, 15, 20, 25, 30, 40, 60, 80, 100)
# -t is the test number ([0-49])

# This command will visualize the result of the 0th instance of the CircleEnv with 10 obstacle density and 60 agents.
python visualizer.py -m CircleEnv -o 10 -r 60 -t 0
```