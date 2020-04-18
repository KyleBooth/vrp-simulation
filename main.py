## Vehicle Routing Problem (VRP): Lightweight Simulator
## Author: Kyle E. C. Booth, kbooth@mie.utoronto.ca

## Python: v3.5.2
## OR-Tools: v7.5

## Required packages
import os
import random
import turtle
import time
import sys
from turtle import Turtle
import threading 
import math
from pprint import pprint
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class robot(Turtle):
     def __init__(self, speed, depot, capacity, shape):
            Turtle.__init__(self, shape)
            self.speed = speed
            self.depot = depot
            self.capacity = capacity
            self.pencolor(random.random(), random.random(), random.random())
            self.pensize(3)
            
class task():
    def __init__(self):
        self.x = (random.random()*2 - 1)*300
        self.y = (random.random()*2 - 1)*300
        
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

def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data["distance_matrix"][from_node][to_node]


def simulate(globalParams, robotParams, taskParams, globalRobotPaths, globalTaskList):
    
    globalNodeTeam = []
    for cycle in range(globalParams["cycles"]):
        nodeTeam = [robot(taskParams["speed"], 0, 0, taskParams["shape"]) for node in range(taskParams["tasks"])]
        globalNodeTeam.append(nodeTeam)
    
    robotTeam = [robot(robotParams["speed"], robotParams["depot"], 
                      + robotParams["capacity"], robotParams["shape"]) for agent in range(robotParams["robots"])]

    ## Define step length for synchronized movement.
    stepLength = 15

    for cycle in range(globalParams["cycles"]):
        ## Disable screen updates.
        turtle.tracer(0)
        for node in range(len(globalNodeTeam[cycle])):
            globalNodeTeam[cycle][node].penup()
            globalNodeTeam[cycle][node].turtlesize(0.6)
            globalNodeTeam[cycle][node].goto(globalTaskList[cycle][node].x, globalTaskList[cycle][node].y)
            globalNodeTeam[cycle][node].write(node)

        ## Re-enable screen updates.
        turtle.tracer(1)
        index = 0
        ## Loop through node placements.
        while index < len(max(globalRobotPaths[cycle], key=len)):
            ## Determine angle to node and turn towards.
            agentAng = []
            for agent in range(len(robotTeam)):
                if index < len(globalRobotPaths[cycle][agent]) and len(globalRobotPaths[cycle][agent]) != 2:
                    ang = robotTeam[agent].towards(globalTaskList[cycle][globalRobotPaths[cycle][agent][index]].x, 
                                                   + globalTaskList[cycle][globalRobotPaths[cycle][agent][index]].y)
                    if (robotTeam[agent].heading()-ang > 0):
                        robotTeam[agent].right(robotTeam[agent].heading()-ang)
                    else:
                        robotTeam[agent].left(ang-robotTeam[agent].heading())
                else:
                    agentAng.append(0)
            ## Initialize step count (10 steps in this case) and distance count for each.
            step = 0
            agentDist = []
            while step < stepLength:
                ## Loop through agents
                for agent in range(len(robotTeam)):
                    if index < len(globalRobotPaths[cycle][agent]) and len(globalRobotPaths[cycle][agent]) != 2:
                        if (step == 0):
                            dist = robotTeam[agent].distance(globalTaskList[cycle][globalRobotPaths[cycle][agent][index]].x, 
                                                             + globalTaskList[cycle][globalRobotPaths[cycle][agent][index]].y)
                            agentDist.append(dist)
                        elif (step == stepLength-1):
                            globalNodeTeam[cycle][globalRobotPaths[cycle][agent][index]].fillcolor("red")
                        ## Make dist/10 move towards goal.
                        robotTeam[agent].forward(agentDist[agent]/stepLength)
                    else:
                        agentDist.append(0)
                step += 1
            ## Proceed to next node.
            index += 1

        for agent in range(len(robotTeam)):
            robotTeam[agent].goto(robotParams["depot"])

        for node in range(len(globalNodeTeam[cycle])):
            globalNodeTeam[cycle][node].fillcolor("green")

        if (cycle != globalParams["cycles"]-1):
            time.sleep(2)
            for agent in range(len(robotTeam)):
                robotTeam[agent].clear()

    turtle.done()

    
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


