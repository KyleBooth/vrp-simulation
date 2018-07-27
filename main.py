## Vehicle Routing Problem (VRP): Lightweight Simulator
## Author: Kyle E. C. Booth
## Date: March 3, 2016

## Required packages
import os
import random
import turtle
from turtle import Turtle
import threading 
import math
from pprint import pprint
from vrp import *

## Problem parameters (should make variable)
globalParams = {"cycles" : 3}
robotParams = {"robots": 3, "speed": 1, "capacity": 10, "depot": (0.00, 0.00), "shape": "triangle"}
taskParams = {"tasks": 10, "speed": 0, "shape": "circle"}

## Robot class definition
class robot(Turtle):
     def __init__(self, speed, depot, capacity, shape):
            Turtle.__init__(self, shape)
            self.speed = speed
            self.depot = depot
            self.capacity = capacity
            self.pencolor(random.random(), random.random(), random.random())
            self.pensize(3)
            
# #Task class definition.
class task():
    def __init__(self):
        self.x = (random.random()*2 - 1)*300
        self.y = (random.random()*2 - 1)*300

# Build distance matrix.
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

if __name__ == '__main__':

	globalTaskList = []
	for cycle in range(globalParams["cycles"]):
    		taskList = [task() for item in range(taskParams["tasks"])]
    		globalTaskList.append(taskList)

	globalTaskList[cycle][0].x = 0
	globalTaskList[cycle][0].y = 0

	## Build the distance matrix
	distance = distances(globalTaskList[cycle])

	maxDistances = []
	for cycle in range(globalParams["cycles"]):
		maxDistance = 0
		i = 0
		while i < taskParams["tasks"]-1:
			maxDistance += distance[i][i+1]
			i += 1
		maxDistances.append(maxDistance)
	
	speed = robotParams["speed"] 
	tasks = taskParams["tasks"]
	robots = robotParams["robots"]
	capacity = robotParams["capacity"]
	max_time = [int(item/2.5) for item in maxDistances] 

	Customer = namedtuple("Customer", ['index', 'x', 'y'])

	customers = []
	for cycle in range(globalParams["cycles"]):
		customers.append([])

	globalRobotPaths = []
	for cycle in range(globalParams["cycles"]):

		print (max_time[cycle])
		print ("customer_count", tasks)
		print ("vehicle_count", robots)

		#Generate randomly distributed customers
		for i in range(tasks):
			customer = Customer(i, globalTaskList[cycle][i].x, globalTaskList[cycle][i].y)
			customers[cycle].append(customer)
		    
		routing = pywrapcp.RoutingModel(tasks, robots, 0)

		search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    		#search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

		routing.SetCost(length)

		routing.AddDimension(time_plus_service_time, max_time[cycle], max_time[cycle], True, "Time")
		time_dimension = routing.GetDimensionOrDie("Time")

		for i in range(1, routing.nodes()):
			routing.AddToAssignment(time_dimension.TransitVar(i))
			routing.AddToAssignment(time_dimension.SlackVar(i))

		assignment = routing.SolveWithParameters(search_parameters)

		# Inspect solution.
		print ("routing.vehicles:", routing.vehicles())
		routes = []
		routeSolution = []
		for i in range(0,routing.vehicles()):
			route_number = i
			routes.append([])
			node = routing.Start(route_number)
			route = []
			route.append(0)

			if routing.IsVehicleUsed(assignment, i):
				while True:
					node = assignment.Value(routing.NextVar(node))
				if not routing.IsEnd(node):
					route.append(int(node))
				else:
					break

			route.append(0)
			routes[route_number].append(route)
			routeSolution.append(route)
		routes=routes[0]

		## Optional print statement for more info.
		#print collect_cumulative_dimension_at_node(routing, assignment, routes, dimension="Time")

		globalRobotPaths.append(routeSolution)
		    
	print (globalRobotPaths)
	
	

	

	


