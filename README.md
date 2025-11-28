# Structure to control two turtles with Turtlesim through ROS2

This repository contains the code for a turtlesim-based interface that allows to control two turtles with a text interface. The interface allows the user to input a turtle of their choice and the respective linear 
and angular velocities and the chosen turtle will move for one second in the given direction, which is published in a specific ROS2 publisher, then stop. During its movement, the chosen turtle will stop if either it 
travels too close to a window boundary or to the other turtle. Once the publishing is over the text interface appears again for a new prompt, and this goes on until a manual interruption through terminal.

## Code explanation and structure
The package is composed by three main folders, a CMake list and a package.xml file; the last two files are both needed for the system uses together C++ source code and a python script to function in its own entirety.

The three main folders are:

- assignment1_rt, which contains the __init__.py script necessary for the Python script to work;
- scripts, which contains the file turtle_spawn.py. This file creates a second turtle in an already initialized turtlesim window, and the position of said turtle can be edited if needed for specific test cases;
- src, which contains the C++ files TurtleController.cpp and TextInterface.cpp. These two files create ROS2 nodes that communicate with each other in moving the selected turtle: TextInterface allows the user to input
  one of two turtles and the respective linear and angular velocities, then the given numbers are published in a common topic between the two programmes and retrieved by TurtleController, which periodically publishes
  the distances between the two turtles and checks the turtles' positions for eventual position violations.

## How to run
Requirements: ROS2 setup with Turtlesim installed 

Procedure to run the system:
- Open four terminals, then run the following command sequence on every terminal: colcon build -> cd install -> source local_setup.bash -> cd ..
- The terminal in which you run the Turtlesim window MUST refer to your ROS2 workspace's root folder
- To run TurtleSim, use ros2 run turtlesim turtlesim_node. To run the other programmes, use ros2 run assignment1_rt <filename> instead on the respective terminal.
- While running, use the terminal in which you run TextInterface to begin experimenting.
- Enjoy!


## Author 
Sisani Francesco
