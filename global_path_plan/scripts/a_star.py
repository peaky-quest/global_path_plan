#! /usr/bin/env python
import rospy
import math
def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
  """
  Identifies neighbor nodes inspecting the 8 adjacent neighbors
  Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
  Returns a list with valid neighbour nodes as [index, step_cost] pairs
  """
  neighbors = []
  # length of diagonal = length of one side by the square root of 2 (1.41421)
  diagonal_step_cost = orthogonal_step_cost * 1.41421
  # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
  lethal_cost = 1

  upper = index - width
  if upper > 0:
    if costmap[upper] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[upper]/255
      neighbors.append([upper, step_cost])

  left = index - 1
  if left % width > 0:
    if costmap[left] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[left]/255
      neighbors.append([left, step_cost])

  upper_left = index - width - 1
  if upper_left > 0 and upper_left % width > 0:
    if costmap[upper_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_left]/255
      neighbors.append([index - width - 1, step_cost])

  upper_right = index - width + 1
  if upper_right > 0 and (upper_right) % width != (width - 1):
    if costmap[upper_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_right]/255
      neighbors.append([upper_right, step_cost])

  right = index + 1
  if right % width != (width + 1):
    if costmap[right] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[right]/255
      neighbors.append([right, step_cost])

  lower_left = index + width - 1
  if lower_left < height * width and lower_left % width != 0:
    if costmap[lower_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_left]/255
      neighbors.append([lower_left, step_cost])

  lower = index + width
  if lower <= height * width:
    if costmap[lower] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[lower]/255
      neighbors.append([lower, step_cost])

  lower_right = index + width + 1
  if (lower_right) <= height * width and lower_right % width != (width - 1):
    if costmap[lower_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_right]/255
      neighbors.append([lower_right, step_cost])

  return neighbors

def calculate_heuristic(node, goal_index, resolution, width):
    """
    Helper function to calculate the heuristic (estimated cost to reach the goal) for a given node.
    You need to implement this function according to your specific environment.
    """

    # Extract the coordinates of the current node
    node_x, node_y = node % width, node // width

    # Extract the coordinates of the goal node
    goal_x, goal_y = goal_index % width, goal_index // width

    # Calculate the Euclidean distance between the current node and the goal node
    dx = (goal_x - node_x) * resolution
    dy = (goal_y - node_y) * resolution
    distance = math.sqrt(dx**2 + dy**2)

    return distance


def a_star(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
    ''' 
    Performs A* search algorithm on a costmap with a given start and goal node
    '''

    # Create an open_list
    open_list = []

    # Set to hold already processed nodes
    closed_list = set()

    # Dict for mapping children to parent
    parents = dict()

    # Dict for mapping g costs (travel costs) to nodes
    g_costs = dict()

    # Set the start node's g_cost
    g_costs[start_index] = 0

    # Add start node to open list with an initial f_cost of 0
    open_list.append([start_index, 0])

    shortest_path = []
    path_found = False
    print('A*: Done with initialization')

    # Main loop, executes as long as there are still nodes inside open_list
    while open_list:

        # Sort open_list according to the lowest 'f_cost' value (second element of each sublist)
        open_list.sort(key=lambda x: x[1])
        # Extract the first element (the one with the lowest 'f_cost' value)
        current_node = open_list.pop(0)[0]

        # Close current_node to prevent from visiting it again
        closed_list.add(current_node)

        # Optional: Visualize closed nodes
        grid_viz.set_color(current_node, "pale yellow")

        # If current_node is the goal, exit the main loop
        if current_node == goal_index:
            path_found = True
            break

        # Get neighbors of current_node
        neighbors = find_neighbors(current_node, width, height, costmap, resolution)

        # Loop neighbors
        for neighbor_index, step_cost in neighbors:

            # Calculate g_cost of neighbor considering it is reached through current_node
            g_cost = g_costs[current_node] + step_cost

            # Calculate h_cost (heuristic cost) for neighbor
            h_cost = calculate_heuristic(neighbor_index, goal_index, resolution, width)

            # Calculate f_cost (g_cost + h_cost) for neighbor
            f_cost = g_cost + h_cost

            # Check if the neighbor has already been visited
            if neighbor_index in closed_list:
                continue

            # Check if the neighbor is in open_list
            in_open_list = False
            for idx, element in enumerate(open_list):
                if element[0] == neighbor_index:
                    in_open_list = True
                    break

            # CASE 1: neighbor already in open_list
            if in_open_list:
                if g_cost < g_costs[neighbor_index]:
                    # Update the node's g_cost, h_cost, f_cost, and parent inside the respective dictionaries
                    g_costs[neighbor_index] = g_cost
                    parents[neighbor_index] = current_node
                    open_list[idx] = [neighbor_index, f_cost]

            # CASE 2: neighbor not in open_list
            else:
                # Set the node's g_cost, h_cost, f_cost, and parent inside the respective dictionaries
                g_costs[neighbor_index] = g_cost
                parents[neighbor_index] = current_node
                # Add neighbor to open_list with its f_cost
                open_list.append([neighbor_index, f_cost])

                # Optional: Visualize frontier
                grid_viz.set_color(neighbor_index, 'orange')

    print('A*: Done traversing nodes in open_list')

    if not path_found:
        print('A*: No path found!')
        return shortest_path

    # Reconstruct path by working backwards from target
    if path_found:
        node = goal_index
        shortest_path.append(goal_index)
        while node != start_index:
            shortest_path.append(node)
            # Get next node
            node = parents[node]
    # Reverse the list
    shortest_path = shortest_path[::-1]
    print('A*: Done reconstructing path')

    return shortest_path
