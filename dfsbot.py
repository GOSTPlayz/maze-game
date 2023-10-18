<<<<<<< HEAD
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
=======

import sys
import select
import time
import random


# current exact position
x = 0.5
y = 0.5

# target in-tile position 
tile_x = 0.5
tile_y = 0.5

# last tile "reached" (i.e., being close enough to center)
ty = -1
tx = -1

# set of walls known
walls = set()
for i in range(0,11):
  walls |= {(i,0,i+0,0), (i,11,i+1,11), (0,i,0,i+1), (11,i,11,i+1)}

# DFS tree
plan = []
seen = set()
dead = set()

# introduce ourselves, all friendly like
print("himynameis DFS-bot", flush=True)

# Wait a few seconds for some initial sense data
time.sleep(0.25)

while True:
  # while there is new input on stdin:
  while select.select([sys.stdin,],[],[],0.0)[0]:
    # read and process the next 1-line observation
    obs = sys.stdin.readline()
    obs = obs.split(" ")
    if obs == []: pass
    elif obs[0] == "bot":
      # update our own position
      x = float(obs[1])
      y = float(obs[2])
      # print("comment now at: %s %s" % (x,y), flush=True)
      # update our latest tile reached once we are firmly on the inside of the tile
      if ((int(x) != tx or int(y) != ty) and
          ((x-(int(x)+tile_x))**2 + (y-(int(y)+tile_y))**2)**0.5 < 0.2):
        tx = int(x)
        ty = int(y)
        if plan == []:
          plan = [(tx,ty)]
          seen = set(plan)
        #print("comment now at tile: %s %s" % (tx,ty), flush=True)
    elif obs[0] == "wall":
      #print("comment wall: %s %s %s %s" % (obs[1],obs[2],obs[3],obs[4]), flush=True)
      # ensure evetile_y wall we see is tracked in our walls set
      x0 = int(float(obs[1]))
      y0 = int(float(obs[2]))
      x1 = int(float(obs[3]))
      y1 = int(float(obs[4]))
      walls |= {(x0,y0,x1,y1)}

  # if we've achieved our goal, update our plan and issue a new command
  if len(plan) > 0 and plan[-1] == (tx,ty):
    # seen == set() means we have not started or are backtracking to the start
    # if len(plan)==1, we're ready to restart again
    if seen > set() or len(plan) == 1:
      seen |= {(tx,ty)}

    # if we've hit our opposing corner:
    if abs(plan[-1][0]-plan[0][0]) == abs(plan[-1][1]-plan[0][1]) == 10:
      # mark all other tiles dead, this is our path, backtrack
      planset = set(plan)
      for i in range(11):
        for j in range(11):
          if (i,j) not in planset:
            dead |= {(i,j)}
      # reset seen so we backtrack path to origin
      seen = set()

    # if pathing, search through lower, right, top, left children, in that order
    # assumes sufficient sense data, but that may not strictly be true  
    if len(seen) > 0 and (tx,ty+1) not in dead|seen and (tx,ty+1,tx+1,ty+1) not in walls:
      plan.append((tx,ty+1))  # move down
      tile_x = 0.5
      tile_y = 0.2
    elif len(seen) > 0 and (tx+1,ty) not in dead|seen and (tx+1,ty,tx+1,ty+1) not in walls:
      plan.append((tx+1,ty))  # move right
      tile_x = 0.2
      tile_y = 0.5
    elif len(seen) > 0 and (tx,ty-1) not in dead|seen and (tx,ty,tx+1,ty) not in walls:
      plan.append((tx,ty-1))  # move up
      tile_x = 0.5
      tile_y = 0.8
    elif len(seen) > 0 and (tx-1,ty) not in dead|seen and (tx,ty,tx,ty+1) not in walls:
      plan.append((tx-1,ty))  # move left
      tile_x = 0.8
      tile_y = 0.5
    else:
    # if we cannot advance or are not pathing currently, backtrack
      dead |= {(tx,ty)}
      plan = plan[:-1]
      tile_x = 0.5
      tile_y = 0.5
      # backtrack further, in one command, if we're
      #   returning to start AND its in a straight line:
      #while seen == set() and (plan[-1][0] == tx or plan[-1][1] == ty):
      #  plan = plan[:-1]
      
    # issue a command for the latest plan
    print("toward %s %s" % (plan[-1][0]+tile_x, plan[-1][1]+tile_y), flush=True)
  
  print("", flush=True)
  time.sleep(0.125)
  
# trying to enhance the dfsbot and make bot move to the edge of the tile : 
# e.g: start         pos = 0.5 , 0.5
#      move right    pos = 1.2 , 0.5
#      move down     pos = 1.5 , 1.2

#  when back track just move back to center of tile to avoid extra calculation.


  
>>>>>>> e5b1764b2b09c60e73489f51ed31d6448ac552e7
