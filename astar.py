import sys
import select
import time

# Define a grid size
GRID_SIZE = 11

# Define the heuristic function (Manhattan distance)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Define a class for nodes in the A* search
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # cost to reach this node from the start
        self.h = 0  # estimated cost from this node to the goal

def astar_search(start, goal, walls):
    open_list = []
    closed_list = set()

    start_node = Node(start)
    goal_node = Node(goal)

    open_list.append(start_node)

    while open_list:
        try:
            # Find the node with the lowest f cost
            current_node = min(open_list, key=lambda node: node.g + node.h)

            # Remove the current node from the open list
            open_list.remove(current_node)
            closed_list.add(current_node.position)

            # If we have reached the goal, reconstruct the path
            if current_node.position == goal_node.position:
                path = []
                while current_node is not None:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Return the path in reverse order

            # Generate child nodes
            for next_position in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                node_position = (
                    current_node.position[0] + next_position[0],
                    current_node.position[1] + next_position[1],
                )

                if (
                    node_position[0] < 0
                    or node_position[0] >= GRID_SIZE
                    or node_position[1] < 0
                    or node_position[1] >= GRID_SIZE
                ):
                    continue

                if node_position in walls:
                    continue

                new_node = Node(node_position, parent=current_node)
                if new_node.position in closed_list:
                    continue

                new_node.g = current_node.g + 1
                new_node.h = heuristic(new_node.position, goal_node.position)

                if any(node.position == new_node.position for node in open_list):
                    existing_node = next(
                        node for node in open_list if node.position == new_node.position
                    )
                    if new_node.g < existing_node.g:
                        existing_node.g = new_node.g
                else:
                    open_list.append(new_node)

        except Exception as e:
            print(f"Error in A* search: {str(e)}")
            break

    return []  # No path found

# Define the initial position
x, y = 0.5, 0.5

# Define the initial goal
goal = (10, 10)

# Define the known walls
walls = set()

# Introduce the bot
print("hi, my name is A* Bot", flush=True)

# Wait for initial sense data
time.sleep(0.25)

while True:
    try:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            obs = sys.stdin.readline()
            obs = obs.split(" ")
            if obs == []:
                pass
            elif obs[0] == "bot":
                x = float(obs[1])
                y = float(obs[2])
            elif obs[0] == "wall":
                x0, y0, x1, y1 = int(float(obs[1])), int(float(obs[2]), int(float(obs[3])), int(float(obs[4])))
                walls.add((x0, y0, x1, y1))

        path = astar_search((int(x), int(y)), goal, walls)
        if path:
            next_x, next_y = path[0]
            print("toward {} {}".format(next_x + 0.5, next_y + 0.5), flush=True)

        time.sleep(0.125)
    except Exception as e:
        print(f"Error in main loop: {str(e)}")
