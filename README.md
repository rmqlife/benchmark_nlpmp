# Benchmark for NLP + motion planning

This is a ROS catkin package for executing and evaluating our Icremental Trajectory Optimization-based Motion Planning (ITOMP) integrated with Natural Language Processing (NLP).

What this package does:

1. It defines config files for obstacles, camera, robot position, parameters, etc.
2. ROS nodes set up environment, send NL commands, receive motion planning results, and execute the planned trajectory.
3. ROS launch files help loading config files and running nodes.

## Requirements

ROS catkin

## Benchmark 1: Laptop

* config/benchmark\_laptop.yaml
* launch/benchmark\_laptop.launch
* src/benchmark\_laptop.cpp, include/benchmark\_laptop.h
