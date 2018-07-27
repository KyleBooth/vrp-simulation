## Simulation
## Build the robot team (list comprehension)
import time

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
                    ## On first calculation, log distance to goal.
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

    #robotTeam[agent].goto(taskList[robotPaths[agent][index]].x, taskList[robotPaths[agent][index]].y)
    for agent in range(len(robotTeam)):
        robotTeam[agent].goto(robotParams["depot"])
    
    for node in range(len(globalNodeTeam[cycle])):
        globalNodeTeam[cycle][node].fillcolor("green")
    
    if (cycle != globalParams["cycles"]-1):
        time.sleep(2)
        for agent in range(len(robotTeam)):
            robotTeam[agent].clear()

turtle.done()

