import re

import numpy as np


def parse_data(data):
    agents = {}
    no_path_agents = []  # 경로가 없는 에이전트를 저장할 리스트
    for line in data:
        match = re.match(r'Agent (\d+):', line)
        if match:
            agent_id = int(match.group(1))
            points = re.findall(r'\(([^)]+)\)', line)
            if points:
                agents[agent_id] = [tuple(map(float, point.split(','))) for point in points]
            else:
                no_path_agents.append(agent_id)  # 경로가 없으면 리스트에 추가
    return agents, no_path_agents


mapname = "CircleEnv"
obs = "20"
robotnum = "30"

sum_of_cost_list = []
sum_of_distance_list = []
makespan_list = []
runtime_list = []

for testnum in range(0, 50):
    # print("Test " + str(testnum) + ":")
    testnum = str(testnum)
    benchmarkPath = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum + "_" + testnum + ".yaml";
    solutionPath = "cbs_solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum + "_" + testnum + "_solution.txt";
    dataPath = "cbs_data/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum + "_" + testnum + "_data.txt";

    # open file from dataPath
    try:
        f = open(dataPath, 'r')
    except FileNotFoundError:
        print("File " + dataPath + " not found!")
        continue
    # read lines from file
    lines = f.readlines()
    # lines = ['352.401,74.3383,0.313497\n']
    sum_of_cost = lines[0].split(',')[0]
    makespan = lines[0].split(',')[1]
    runtime = lines[0].split(',')[2].strip('\n')
    if sum_of_cost == "0":
        print("File " + dataPath + " is empty!")
        continue

    sum_of_cost_list.append(float(sum_of_cost))
    makespan_list.append(float(makespan))
    runtime_list.append(float(runtime))

    # print(sum_of_cost)
    # print(makespan)
    # print(runtime)

    # close file
    f.close()

    # open file from solutionPath
    try:
        f = open(solutionPath, 'r')
    except FileNotFoundError:
        print("File " + solutionPath + " not found!")
        continue
    # read lines from file
    lines = f.readlines()
    agents, no_path_agents = parse_data(lines)
    # print(f'Agents without a path: {no_path_agents}')
    # print(f'Number of agents: {len(agents)}')

    # calculate the distance of path of each agent
    distance_list = []
    for agent_id, path in agents.items():
        distance = 0
        for i in range(len(path) - 1):
            x0, y0, t0 = path[i]
            x1, y1, t1 = path[i + 1]
            distance += np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        distance_list.append(distance)

    sum_of_distance_list.append(sum(distance_list))
    # print("Sum of distance: " + str(sum(distance_list)))

    # close file
    f.close()

if len(sum_of_cost_list) == 0:
    print("No data found!")
else:
    print("Average sum of cost: " + str(sum(sum_of_cost_list) / len(sum_of_cost_list)))
    print("Average sum of distance: " + str(sum(sum_of_distance_list) / len(sum_of_distance_list)))
    print("Average makespan: " + str(sum(makespan_list) / len(makespan_list)))
    print("Average runtime: " + str(sum(runtime_list) / len(runtime_list)))
    print("Std of sum of cost: " + str(np.std(sum_of_cost_list)))
    print("Std of sum of distance: " + str(np.std(sum_of_distance_list)))
    print("Std of makespan: " + str(np.std(makespan_list)))
    print("Std of runtime: " + str(np.std(runtime_list)))
    print("Success rate: " + str(len(sum_of_cost_list) / 50 * 100) + "%")
