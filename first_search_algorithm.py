# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

grid2 = [[0, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0]]
        
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']
        
def search(grid,init,goal,cost):
    # check adjacent cells for presence of 0 or 1
    # if 0, determine g value from init
    # choose lowest g value and expand
    # keep expanding the lowest g value until goal grid is reached
    # if goal grid cannot be reached, return "error".
    # default open list to init co-ordinates and g = 0.
    path = "Tester - still not long enough."
    open_list = []
    open_list.append([0, init[0], init[1]])
    closed_cells = []
    closed_cells_g = []
    goal_found = False
    while goal_found == False:
        if len(open_list) == 0:
            path = "fail"
            goal_found = True
        # determine lowest g value from open_list, copy it and remove from open_list
        open_list = sorted(open_list, key=lambda x: x[0])
        g_low = open_list.pop(0)
        adjacent = []
        left = False if g_low[1] == 0 else True
        right = False if g_low[1] == len(grid)-1 else True
        up = False if g_low[2] == 0 else True
        down = False if g_low[2] == len(grid[0])-1 else True
        if left:
            adjacent.append([g_low[0] + cost, g_low[1] - 1, g_low[2]])
        if right:
            adjacent.append([g_low[0] + cost, g_low[1] + 1, g_low[2]])
        if down:
            adjacent.append([g_low[0] + cost, g_low[1], g_low[2] + 1])
        if up:
            adjacent.append([g_low[0] + cost, g_low[1], g_low[2] -1])
        if adjacent:
            for cell in adjacent:
                if [cell[1], cell[2]] in closed_cells:
                    continue
                if grid[cell[1]][cell[2]] == 1:
                    continue
                open_list.append([cell[0], cell[1], cell[2]])
                if [cell[1], cell[2]] == goal:
                    path = [cell[0], cell[1], cell[2]]
                    goal_found = True
            closed_cells.append([g_low[1], g_low[2]])
            closed_cells_g.append([g_low[0], g_low[1], g_low[2]])
        else: 
            return "fail"
    return path

print search(grid, init, goal, cost)