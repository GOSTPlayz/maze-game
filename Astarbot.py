
import sys
import select
import time
import random
import copy, queue

# current exact position
x = 0.5
y = 0.5

# flag tile position
flag_x = 10
flag_y = 10

# last tile "reached" (i.e., being close enough to center)
ty = -1
tx = -1

# set of walls known
walls = set()
for i in range(0,11):
  walls |= {(i,0,i+0,0), (i,11,i+1,11), (0,i,0,i+1), (11,i,11,i+1)}

# A*
plan = []
seen = set()
dead = set()

class G5Bot:
  def __init__(self, x , y):
    self.x = x
    self.y = y
    self.plan = []
    self.path_cost = 0
  
  def actions(self):
    action_set = []
    if (self.x,self.y+1) not in dead|seen and (self.x,self.y+1,self.x+1,self.y+1) not in walls:
      action_set.append((self.x,self.y+1))  # move down
    if (self.x+1,self.y) not in dead|seen and (self.x+1,self.y,self.x+1,self.y+1) not in walls:
      action_set.append((self.x+1,self.y))  # move right
    if (self.x,self.y-1) not in dead|seen and (self.x,self.y,self.x+1,self.y) not in walls:
      action_set.append((self.x,self.y-1))  # move up
    if (self.x-1,self.y) not in dead|seen and (self.x,self.y,self.x,self.y+1) not in walls:
      action_set.append((self.x-1,self.y))  # move left
    return action_set
  
  def speculate_move(self, cx, cy): #it not actualy moving, just plan the move
    speculate = copy.deepcopy(self) 
    speculate.x = cx
    speculate.y = cy
    speculate.path_cost = self.path_cost + 1
    speculate.plan.append((cx, cy))
    return speculate

  def successors(self):
    successor_set = []
    for action in self.actions():
      successor_set.append(self.speculate_move(action[0], action[1]))
    return successor_set
  
  def __lt__(self, other):
    return self.eval() < other.eval()
  
  def goal_distance(self):
    distance = abs(flag_x - self.x) + abs(flag_y - self.y)
    return distance
  
  def eval(self): #evaluate 
    return  self.path_cost + self.goal_distance()


def planning(tup):
  global dead
  plan_bot = G5Bot(tup[0], tup[1])
  frontier = queue.PriorityQueue()
  frontier.put(plan_bot)
  while not frontier.empty():
    current_speculate = frontier.get()
    if current_speculate.goal_distance() == 0:
      return current_speculate
    
    if len(current_speculate.actions()) == 0: 
      dead |= {(current_speculate.x, current_speculate.y)}
      continue
    
    for successor in current_speculate.successors():
      frontier.put(successor)
  return None
      
def invalid_move(from_pos, to_pos): 
  fx, fy = from_pos[0], from_pos[1]
  tx, ty = to_pos[0], to_pos[1]
   
  if (ty - fy == 1) and (fx,fy+1,fx+1,fy+1) in walls:
    return True
  elif (tx - fx == 1) and (fx+1,fy,fx+1,fy+1) in walls:
    return True
  elif (ty - fy == -1) and (fx,fy,fx+1,fy) in walls:
    return True
  elif (tx - fx == -1) and (fx,fy,fx,fy+1) in walls:
    return True
  else:
    return False
  
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
          ((x-(int(x)+0.5))**2 + (y-(int(y)+0.5))**2)**0.5 < 0.2):
        tx = int(x)
        ty = int(y)
        if plan == []:
          plan = [(tx,ty)]
          seen = set(plan)
        #print("comment now at tile: %s %s" % (tx,ty), flush=True)
    elif obs[0] == "wall":
      #print("comment wall: %s %s %s %s" % (obs[1],obs[2],obs[3],obs[4]), flush=True)
      # ensure every wall we see is tracked in our walls set
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

    if len(seen) > 0:
      G5_bot = planning((tx, ty))
      if G5_bot is not None: 
        plan.append((G5_bot.plan[0]))
      else : # we discover new wall that make the current path a dead-end
        seen.discard(plan[-1])
        plan = plan[:-1]# backtrack
        
    print("toward %s %s" % (plan[-1][0]+0.5, plan[-1][1]+0.5), flush=True)
  elif len(plan) > 1 and (invalid_move(plan[-2],plan[-1])): #made invalid move:
    plan = plan[:-1]
  print("", flush=True)
  time.sleep(0.125)
  

# The bot can observe 2 tiles away (not consistent) the goal is to early prune the seen path before move
# to the next tile.
# bot will plan the next move base on the current 2 tiles observation (asume know all the walls)
# If bot see a dead end it will go back without have to move around the dead end
#  