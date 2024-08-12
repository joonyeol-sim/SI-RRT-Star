import re
import numpy as np
import csv

def parse_data(data):
    agents = {}
    no_path_agents = []
    for line in data:
        match = re.match(r'Agent (\d+):', line)
        if match:
            agent_id = int(match.group(1))
            points = re.findall(r'\(([^)]+)\)', line)
            if points:
                agents[agent_id] = [tuple(map(float, point.split(','))) for point in points]
            else:
                no_path_agents.append(agent_id)
    return agents, no_path_agents

def process_robotnum(mapname, obs, robotnum):
    sum_of_cost_list = []
    sum_of_distance_list = []
    makespan_list = []
    runtime_list = []

    for testnum in range(0, 50):
        testnum = str(testnum)
        dataPath = f"over_data/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_data.txt"
        solutionPath = f"over_solution/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_solution.txt"

        try:
            with open(dataPath, 'r') as f:
                lines = f.readlines()
                if not lines or lines[0].strip() == "0":
                    continue
                sum_of_cost, makespan, runtime = map(float, lines[0].strip().split(','))
                sum_of_cost_list.append(sum_of_cost)
                makespan_list.append(makespan)
                runtime_list.append(runtime)

            with open(solutionPath, 'r') as f:
                lines = f.readlines()
                agents, _ = parse_data(lines)
                distance_list = []
                for path in agents.values():
                    distance = sum(np.sqrt((x1-x0)**2 + (y1-y0)**2) for (x0,y0,_), (x1,y1,_) in zip(path, path[1:]))
                    distance_list.append(distance)
                sum_of_distance_list.append(sum(distance_list))

        except FileNotFoundError:
            continue

    if not sum_of_cost_list:
        return None

    return {
        "Sum of cost avg (time)": np.mean(sum_of_cost_list),
        "Sum of cost avg (distance)": np.mean(sum_of_distance_list),
        "Makespan avg": np.mean(makespan_list),
        "Runtime avg": np.mean(runtime_list),
        "Sum of cost std (time)": np.std(sum_of_cost_list),
        "Sum of cost std (distance)": np.std(sum_of_distance_list),
        "Makespan std": np.std(makespan_list),
        "Runtime std": np.std(runtime_list),
        "Success rate": len(sum_of_cost_list) / 50
    }

def main():
    mapname = "RectEnv"
    obs = "20"
    robot_nums = [120, 140, 160, 180, 200]

    results = []
    for robotnum in robot_nums:
        result = process_robotnum(mapname, obs, str(robotnum))
        if result:
            result["Algorithms"] = "ST-RRT"
            result["Environment"] = mapname + " " + obs
            result["Agent num"] = robotnum
            results.append(result)

    # Writing to CSV
    fieldnames = ["Algorithms", "Environment", "Agent num", "Sum of cost avg (time)", "Sum of cost avg (distance)",
                  "Makespan avg", "Runtime avg", "Sum of cost std (time)", "Sum of cost std (distance)",
                  "Makespan std", "Runtime std", "Success rate"]

    with open('multi_robot_analysis_results.csv', 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            writer.writerow(result)

    print("Results have been written to multi_robot_analysis_results.csv")

if __name__ == "__main__":
    main()