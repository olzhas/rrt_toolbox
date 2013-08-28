% MATLAB RRT*FN package for solving path/motion planning problems.
% Version 0.0.1 23-Aug-2013
%
% RRT and RRT* implementations are included as well
%
% FILES:
%   README.md                        - readme file, read it please.
%   rrt                              - Rapidly-Exploring Random Tree (RRT)
%   rrt_star                         - RRT*
%   rrt_star_fn                      - RRT*FN (Fixed Nodes)
%   FNRedundantManipulator           - Model of "n" DOF planar redundant
%                                      manipulator.
%   FNSimple2D                       - Model of 2D mobile robot
%   GridBased2Dimrotate              - Initial work on model of KUKA youbot
%                                      GPU usage employed (faster)
%   benchmark2D                      - benchmark script for 2D mobile robot
%   benchmarkRedundant               - benchmark script for nDOF redundant planar manipulator
%
% CONFIGURATION FILES:
%   configure_FNRedundantManipulator - Default configuration for Redundant Robotic Manipulator nDOF
%   configure_FNSimple2D             - Default configuration for 2D case
%   configure_GridBased2Dimrotate    - Default configuration for 2D case
