# ROS_Global_Path_Plan

# Path Planning Repository
This repository contains packages and code that are helpful for path planning using the A* and Dijkstra algorithms. Additionally, custom global server and client scripts are provided to control the movement of the TurtleBot according to the path generated by the implemented path planning algorithms

# Contents

The repository consists of the following packages:

global_path_plan: This package provides the simulation environment for path planning on Gazebo. It includes the mbot_simulation.launch file, which is used to run the simulation on Gazebo. The simulation environment allows you to test and visualize the path planning algorithms using a virtual robot.

kobuki_description: This package contains the description files for the Kobuki robot. It includes the necessary URDF and mesh files to visualize and simulate the Kobuki robot in ROS.

turtlebot_description: This package contains the description files for the TurtleBot robot. It includes the necessary URDF and mesh files to visualize and simulate the TurtleBot robot in ROS.

# Path Planning Algorithms

The A* and Dijkstra algorithms are widely used for path planning in robotics and computer science. Both algorithms aim to find the shortest path between a given start and goal location.
A* Algorithm

The A* (pronounced "A-star") algorithm is an informed search algorithm that efficiently searches for the optimal path by using heuristics to guide the search process. It considers both the actual cost of reaching a node from the start position (known as g-value) and an estimate of the remaining cost to reach the goal (known as h-value). By considering these two factors, A* can efficiently find the optimal path with the least cost.
Dijkstra Algorithm

The Dijkstra algorithm, also known as the shortest path first algorithm, is an algorithm that finds the shortest path between nodes in a graph. It does not use heuristics like A*, but instead iteratively selects the node with the smallest cost and expands its neighbors until the goal node is reached. Although the Dijkstra algorithm guarantees finding the shortest path, it may be slower than A* for large graphs due to the lack of heuristics.


# Installation and Usage

To use the packages in this repository, please follow these steps:

1. Clone the repository to your local machine:
2. Install the required dependencies for the path planning and TurtleBot control packages. Ensure that you have the necessary dependencies and libraries installed to successfully run the code.
3. Build the packages using the appropriate build system for your ROS installation
4. To run the simulation use the folowing command
   $ cd catkin_ws
   $ catkin_make
   $ source devel/setup.bash
   $ roslaunch global_path_plan mbot_simulation.launch

5. *For A_star launch the*
   roslaunch global_path_plan a_star.launch

6. *For Dijkstra launch the*
   roslaunch global_path_plan dijkstra.launch

# License

This repository is released under the MIT License. Feel free to modify, distribute, and use the code in accordance with the terms and conditions specified in the license.
Issues and Contributions

If you encounter any issues or have suggestions for improvements, please open an issue on the repository's issue tracker. Contributions, such as bug fixes or new features, are welcome and can be submitted via pull requests.

Please ensure that you follow the code of conduct and guidelines for contributing as outlined in the repository.

# Acknowledgments

We would like to acknowledge the contributions of all individuals and projects that have helped in the development of this repository. Their efforts and dedication are greatly appreciated.
