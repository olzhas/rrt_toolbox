MATLAB implementation of RRT, RRT\* and RRT\*FN algorithms.
================================================================

## What is RRT, RRT\* and RRT\*FN

- RRT (Rapidly-Exploring Random Tree) is a sampling-based algorithm for
solving path planning problem. RRT provides feasable solution
if time of RRT tends to infinity.

- RRT\* is a sampling-based algorithm for solving motion planning problem,
which is an probabilistically optimal variant of RRT. RRT* converges to the optimal solution asymptotically.

- RRT\*FN is a sampling-based algorithm based on RRT\*.
RRT\*FN inherents asymptotical convergence to the optimal solution,
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
- Olzhas Adiyatov oadiyatov[at]nu.edu.kz
- Atakan Varol

(c) 2013
