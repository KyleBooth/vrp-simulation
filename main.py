## Vehicle Routing Problem (VRP): Lightweight Simulator
## Author: Kyle E. C. Booth, kbooth@mie.utoronto.ca

import random
import sys
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from simulate import simulate
from helper import *
            
class task():
    def __init__(self):
        self.x = (random.random()*2 - 1)*300
        self.y = (random.random()*2 - 1)*300

def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data["distance_matrix"][from_node][to_node]
    
globalParams = {"cycles" : -1}
robotParams = {"robots": -1, "speed": 1, "capacity": 10, "depot": (0.00, 0.00), "shape": "triangle"}
taskParams = {"tasks": -1, "speed": 0, "shape": "circle"}

if __name__ == '__main__':

    args = []
    for i, arg in enumerate(sys.argv):
        args.append(arg)
   
    globalParams["cycles"] = int(args[1])
    robotParams["robots"] = int(args[2])
    taskParams["tasks"] = int(args[3])

    globalRobotPaths = []
    globalTaskList = []
    for cycle in range(globalParams["cycles"]):
    
        taskList = [task() for item in range(taskParams["tasks"])]
        taskList = setDepot(taskList)

        data = create_data_model(taskList, robotParams["robots"])

        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

        routing = pywrapcp.RoutingModel(manager)

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            3000,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name)
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)

        assignment = routing.SolveWithParameters(search_parameters) 

        if assignment:
            robot_paths = encode_solution(data, manager, routing, assignment)

        globalRobotPaths.append(robot_paths)
        globalTaskList.append(taskList)
        
    simulate(globalParams, robotParams, taskParams, globalRobotPaths, globalTaskList)


