**VRP Simulator:** Python Turtle and Google OR-Tools  
**Author:** Kyle E. C. Booth (kbooth@mie.utoronto.ca) 

This repository contains code base for a lightweight simulator of the multiple traveling salesman problem (mTSP) and vehicle routing problem (VRP). Vehicle movement is simulated using [Python Turtle](https://docs.python.org/3.3/library/turtle.html?highlight=turtle), and vehicle routes are generated using [Google OR-Tools](https://developers.google.com/optimization/routing) routing solver.

### Requirements:

Last tested using Python v3.5 and OR-Tools v7.5.

### File/directory descriptions:

In progress.

### Running the simulator:

Running the simulator is simple. Navigate to the directory and run the following in terminal:

```console
python main.py `<cycles>` `<vehicles>` `<tasks>`
```

* cycles: indicates the number of times a new instance should be generated, solved, and simulated
* vehicles: the number of agents that are routed
* tasks: the number of customers visited across all vehicles

_Note: Recommend using less than 5 vehicles and less than 30 tasks._

A Python Turtle window should pop-up and simulate the solution to the VRP, resembling something like the following:

<img src="img/simulation-demo.png" width="600" align="center">
