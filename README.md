MATLAB implementation of RRT, RRT\* and RRT\*FN algorithms.
================================================================

## What is RRT, RRT\* and RRT\*FN

- RRT (Rapidly-Exploring Random Tree) is a sampling-based algorithm for
solving path planning problem. RRT provides feasable solution
if time of RRT tends to infinity.

- RRT\* is a probabilistic algorithm for solving motion planning problem,
which is similar to RRT but unlike RRT provides faster rate of
convergance to the feasable solution.

- RRT\*FN also is a probabilistic algorithm based on RRT\*.
RRT\*FN inherents faster rate of convergence to the feasable solution,
however RRT\*FN implements it using less memory.

## How to use
The original package contains 3 files containing algorithm

- rrt.m
- rrt\_star.m
- rrt\_star\_fn.m

and 2 files containing classes that enable algorithm to solve
problems for simple 2D mobile robot model and nDOF Redundant Manipulator
- FNSimple2D.m 
- FNRedundantManipulator.m

One can add other additional models implementing all methods mentioned in
rrt.m, rrt\_star.m, rrt\_star\_fn.m


## Authors
- Olzhas Adiyatov
- Atakan Varol

(c) 2013
