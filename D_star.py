
import sys
import select
import time
import random
import copy
import queue


# current exact position
x = 0.5
y = 0.5
home_x,home_y = 0,0
goal_x,goal_y = 10,10 

# last tile "reached" (i.e., being close enough to center)
ty = -1
tx = -1

# number of signals before start:
start_timeout_signals = 10

# set of walls known
processed_walls = set()
new_walls = set()
for i in range(0,11):
  new_walls |= {(i,0,i+0,0), (i,11,i+1,11), (0,i,0,i+1), (11,i,11,i+1)}

# walls functions
def up_wall(x, y):
  return (x,y,x+1,y) 
def down_wall(x, y):   
  return (x,y+1,x+1,y+1)
def right_wall(x, y):
  return (x+1,y,x+1,y+1)
def left_wall(x, y):
  return (x,y,x,y+1)

def update_walls():
  global processed_walls, new_walls
  processed_walls.update(new_walls) # add walls to processed
  new_walls = set() #reset the new walls set

# D*
opt_plan = []
backtrack_plan = []
exec_index = 0
traversed = set()
# seen = set()
dead = set()
gave_up = False

class Plan:
  def __init__(self, x, y, gx, gy):
    self.x = x
    self.y = y
    self.gx = gx
    self.gy = gy
    self.plan = []
    self.cost = 0
  def actions(self):
    actions_set = []
    if (self.x,self.y+1) not in dead|traversed and down_wall(self.x, self.y) not in processed_walls:
      actions_set.append((self.x,self.y+1))  # move down
    if (self.x+1,self.y) not in dead|traversed and right_wall(self.x,self.y) not in processed_walls:
      actions_set.append((self.x+1,self.y))  # move right
    if (self.x,self.y-1) not in dead|traversed and up_wall(self.x, self.y) not in processed_walls:
      actions_set.append((self.x,self.y-1))  # move up
    if (self.x-1,self.y) not in dead|traversed and left_wall(self.x, self.y) not in processed_walls:
      actions_set.append((self.x-1,self.y))  # move left
    return actions_set
  def move(self, action):
    next = copy.deepcopy(self)
    next.x = action[0]
    next.y = action[1]
    next.cost = self.cost + 1
    next.plan.append((action[0],action[1]))
    return next
  
  def successors(self):
    successor_set = []
    for action in self.actions():
      successor_set.append(self.move(action))
    return successor_set
  
  def goal_distance(self):
    distance = abs(self.gx - self.x) + abs(self.gy - self.y)
    return distance
  
  def __lt__(self, other):
    return self.eval() < other.eval()
  
  def eval(self): #evaluate 
    return  self.cost + self.goal_distance()

def planning(tup): # getting optimal plan with A* and mahattan distance heuristic
  global dead, goal_x, goal_y
  planner = Plan(tup[0], tup[1], goal_x, goal_y)
  checked_set = set()
  frontier = queue.PriorityQueue()
  frontier.put(planner)
  while not frontier.empty():
    current_speculation = frontier.get()
    if current_speculation.goal_distance() == 0:
      return current_speculation
    
    checked_set |= {(current_speculation.x, current_speculation.y)}
    
    for successor in current_speculation.successors():
      if (successor.x, successor.y) not in checked_set:
        frontier.put(successor)
  return None

def follow_plan():
  global exec_index, opt_plan
  exec_index += 1
  next_step = opt_plan[exec_index]
  print("toward %s %s" % (next_step[0]+0.5, next_step[1]+0.5), flush=True)
  
def back_track():
  global backtrack_plan
  next_step = backtrack_plan[0]
  print("toward %s %s" % (next_step[0]+0.5, next_step[1]+0.5), flush=True)

def flip_goal():
  global home_x,home_y,goal_y,goal_x
  home_x,goal_x = goal_x,home_x
  home_y,goal_y = goal_y,home_y

# introduce ourselves, all friendly like
print("himynameis DSL-bot", flush=True)

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
      if ((int(x) != tx or int(y) != ty) and ((x-(int(x)+0.5))**2 + (y-(int(y)+0.5))**2)**0.5 < 0.2):
        tx = int(x)
        ty = int(y)
        #print("comment now at tile: %s %s" % (tx,ty), flush=True)
    elif obs[0] == "wall":
      # print("comment wall: %s %s %s %s" % (obs[1],obs[2],obs[3],obs[4]), flush=True)
      x0 = int(float(obs[1]))
      y0 = int(float(obs[2]))
      x1 = int(float(obs[3]))
      y1 = int(float(obs[4]))
      if (x0,y0,x1,y1) not in processed_walls: 
        new_walls |= {(x0,y0,x1,y1)}  #added to new walls first to check if they affect the plan
    # elif obs[0] == "observed":
    #   seen.add((int(obs[1]), int(obs[2])))
  
  # wait for # signals before start
  if start_timeout_signals > 0:
    start_timeout_signals -=1
    pass
  
  #just start (again)
  # if len(opt_plan) == 0 and (tx, ty) in seen:
  if len(opt_plan) == 0 and start_timeout_signals == 0:
    # print("comment start planning", flush=True)
    # start planning
    traversed = {(tx, ty)}
    update_walls()
    planner = planning((tx, ty))
    # initial plan
    if planner is not None and len(planner.plan) > 0:
      opt_plan = [(tx, ty)] + planner.plan
      # print("comment got a plan start moving", flush=True)
      follow_plan()
    else:
      print("comment cannot solve this maze", flush=True)
      gave_up = True # cannot find a path, give up
      pass
  
  # is backtracking
  if len(opt_plan) > 0 and len(backtrack_plan) > 0 and backtrack_plan[0] == (tx,ty):
    # print("comment backtracking", flush=True)
    backtrack_plan = backtrack_plan[1:]
    if len(backtrack_plan) > 0: #keep backtracking
      back_track()
    elif opt_plan[exec_index] == (tx,ty): #back on opt plan path
      follow_plan()
    else:
      print("comment error backtracking", flush=True)
  
  # reach the current executed step.
  elif len(opt_plan) > 0 and opt_plan[exec_index] == (tx,ty) and not gave_up:
    # check if current step is goal:
    if(opt_plan[exec_index][0] == goal_x) and (opt_plan[exec_index][1] == goal_y):
      flip_goal()
      opt_plan = []
      exec_index = 0
      traversed = set()
      continue
    
    # check for new walls:
    if len(new_walls) == 0: # no new walls detected -> execute next step in opt_plan
      # print("comment No new wall moving on", flush=True)
      traversed |= {(tx, ty)}
      follow_plan()
    else: # detected new walls, check if affect the opt_plan
      # print("comment New wall detected", flush=True)
      fault_index = 0
      for i in range(exec_index, len(opt_plan) - 1):
        if (
          ((opt_plan[i+1][0] - opt_plan[i][0] == 1) and right_wall(opt_plan[i][0],opt_plan[i][1]) in new_walls) or 
        	((opt_plan[i+1][1] - opt_plan[i][1] == 1) and down_wall(opt_plan[i][0],opt_plan[i][1]) in new_walls) or
					((opt_plan[i+1][0] - opt_plan[i][0] == -1) and left_wall(opt_plan[i][0],opt_plan[i][1]) in new_walls) or 
					((opt_plan[i+1][1] - opt_plan[i][1] == -1) and up_wall(opt_plan[i][0],opt_plan[i][1]) in new_walls)
        ): 
          fault_index = i
          break
      update_walls()
      if fault_index == 0: #new walls not affect current plan, moving on
        # print("comment New wall not affect plan, moving on", flush=True)
        follow_plan()
      else: # affected. re-planning
        # print("comment New wall affect plan recalibrating", flush=True)
        while True:
          planner = planning(opt_plan[exec_index]) #recalibrate / planning from backtrack
          if planner is not None:
            opt_plan = opt_plan[:exec_index+1:] # cut off the invalid moves
            opt_plan = opt_plan + planner.plan # add the new plan
            break
          else: # cannot solve, try to backtrack (hit dead end)
            # print("comment dead end: %s pos:(%s, %s)" % (fault_index, opt_plan[fault_index][0],opt_plan[fault_index][1]), flush=True)
            while True: 
              current_step = opt_plan[exec_index]
              bt_planner = Plan(current_step[0], current_step[0][1])
              if len(bt_planner.actions()) > 0 or exec_index == 0: # find the last step that can still make a move
                break
              exec_index -= 1
              backtrack_plan.append[opt_plan[exec_index]] # create a backtrack plan
              # traversed.discard(current_step) 
              dead |= {current_step}
            if exec_index == 0: # hit start position, maze unsolvable
              # print("comment cannot solve maze after back track planning", flush=True)
              gave_up = True
            # else: #found different path at move-able pos, planning again
              # print("comment last step before de: %s pos:(%s, %s)" % (exec_index, opt_plan[exec_index][0],opt_plan[exec_index][1]), flush=True)
              # break
        if (tx, ty) == opt_plan[exec_index]: #still on track
          # print("comment finish recalibrating still on track move on", flush=True)
          follow_plan()
        elif len(backtrack_plan) > 0:
          # print("comment finish recalibrating ran to dead end, backtracking", flush=True)
          back_track()
        else:
          print("comment unknown error", flush=True)
  
  print("", flush=True)
  time.sleep(0.1)
  