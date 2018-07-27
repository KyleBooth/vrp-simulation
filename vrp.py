import random
import math
from collections import namedtuple
from ortools.constraint_solver import pywrapcp

def length(customer1, customer2):
    return round(math.sqrt((customers[cycle][customer1][1] - customers[cycle][customer2][1])**2 + 
                           + (customers[cycle][customer1][2]- customers[cycle][customer2][2])**2))

def collect_cumulative_dimension_at_node(routing_model, assignment_solution, routes, dimension="Time"):
    result = list()
    for index, route in enumerate(routes):
        row = list()
        for index, el in enumerate(route[1:-1]):
            cum_time=assignment_solution.Value( routing_model.CumulVar(el,dimension))
            transit_time = 0
            idle_time = 0
            transit_time = assignment_solution.Value(routing_model.TransitVar(el,dimension))
            idle_time = assignment_solution.Value(routing_model.SlackVar(el,dimension))
            row.append((el,cum_time,transit_time,idle_time))
        result.append(row)

    return result

def time_plus_service_time(customer1,customer2):
    return round(length(customer1,customer2)) 



