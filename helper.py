## Vehicle Routing Problem (VRP): Lightweight Simulator
## Author: Kyle E. C. Booth, kbooth@mie.utoronto.ca

import math

def setDepot(taskList):
    taskList[0].x = 0
    taskList[0].y = 0
    return taskList
    
def distances(taskList):
    distanceMatrix = []
    tmp = []
    for i in range(len(taskList)):
        for j in range(len(taskList)):
            if i != j:
                tmp.append(int(math.sqrt((taskList[i].x-taskList[j].x)**2 + (taskList[i].y-taskList[j].y)**2)))
            else:
                tmp.append(0)
        distanceMatrix.append(tmp)
        tmp = []
    return distanceMatrix

def create_data_model(taskList, robots):
    data = {}
    data["distance_matrix"] = distances(taskList)
    data["num_vehicles"] = robots
    data["depot"] = 0
    return data

def encode_solution(data, manager, routing, assignment):
    robot_paths = []
    for vehicle_id in range(data['num_vehicles']):
        robot_path = []
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            robot_path.append(manager.IndexToNode(index))
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
        robot_path.append(manager.IndexToNode(index))
        robot_paths.append(robot_path)
    return robot_paths


