# import queue
import numpy as np
from collections import defaultdict
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def create_world(length, width):
    m = int(length / 0.1) + 1       # number of rows
    n = int(width / 0.1)  + 1       # number of columns

    world_grid = np.zeros((m, n))

    for i in range(m):
        world_grid[i][0] = 1
        world_grid[i][n-1] = 1
    
    for j in range(n):
        world_grid[0][j] = 1
        world_grid[m-1][j] = 1

    return world_grid, m, n

def set_obstacle(world_grid, startx, starty, obLen, obWid):
    l = int(obLen / 0.1) + 1
    w = int(obWid / 0.1) + 1

    world_grid[startx: startx+l, starty:starty+w] = 1

    return world_grid

def heuristic(goal, cur):
    # Heuristic function with Chebyshev distance
    d_diagonal = min(abs(goal[0] - cur[0]), abs(goal[1] - cur[1]))
    d_straight = abs(goal[0] - cur[0]) + abs(goal[1] - cur[1])
    return math.sqrt(2) * d_diagonal + (d_straight - 2 * d_diagonal)

def move_cost(next, cur):
    # Move cost from current point to next point
    return math.sqrt((next[0] - cur[0]) ** 2 + (next[1] - cur[1]) ** 2)

def safety_cost(cur, m, n, world_grid):
    # Safety cost
    d_obstacle = m

    for i in range(m):
        for j in range(n):
            if world_grid[i, j] == 1:
                d_obstacle = min(d_obstacle, move_cost((i, j), cur))

    return 1.0 / (d_obstacle + 1e-5)

def equals(goal, cur):
    return goal[0] == cur[0] and goal[1] == cur[1]

# def Astar_planning(start, goal, m, n, world_grid):
#     directions = [[-1, 0], [0, 1], [-1, 1]]
#     que = queue.PriorityQueue()
#     que.put(start, 0)

#     current_cost = defaultdict(float)
#     current_cost[start] = 0.0

#     came_from = dict()
#     came_from[start] = None

#     while not que.empty():
#         cur = que.get()

#         if equals(goal, cur):
#             break
        
#         for direction in directions:
#             nextX, nextY = cur[0] + direction[0], cur[1] + direction[1]

#             if nextX < 0 or nextX == m-1 or nextY < 0 or nextY == n-1:
#                 continue
        
#             if world_grid[nextX, nextY] == 1:
#                 continue

#             next = (nextX, nextY)
#             new_cost = current_cost[cur] + move_cost(next, cur)

#             if next not in current_cost or new_cost < current_cost[next]:
#                 current_cost[next] = new_cost
#                 priority = new_cost + heuristic(goal, next)
#                 que.put(next, priority)
#                 came_from[next] = cur
#                 # world_grid[next] = priority
    
#     return came_from, world_grid

def greedy_search(start, goal, m, n, world_grid, mw, hw, sw, mode='safety'):
    directions = [[-1, 0], [0, 1], [-1, 1]]
    cur = start
    
    pre_direction = (0, 0)
    cur_direction = pre_direction
    if mode == 'safety':
        path = []
    else:
        path = [cur]

    while not equals(cur, goal):
        if mode != 'safety':
            if cur_direction == pre_direction:
                path[-1] = cur
            else:
                path.append(cur)
        else:
            path.append(cur)
        # path.append(cur)

        pre_direction = cur_direction
        min_cost = float('inf')
        tmp = cur

        for direction in directions:
            nextX, nextY = cur[0] + direction[0], cur[1] + direction[1]

            if nextX < 0 or nextX == m-1 or nextY < 0 or nextY == n-1:
                continue
        
            if world_grid[nextX, nextY] == 1:
                continue

            next = (nextX, nextY)
            estimated_cost = mw * move_cost(next, cur) + hw * heuristic(goal, next) + sw * safety_cost(next, m, n, world_grid)

            if estimated_cost < min_cost:
                min_cost = estimated_cost
                tmp = next
                cur_direction = (direction[0], direction[1])
        cur = tmp
    path.append(goal)
    return path

def generate_safety_path():
    world_grid, m, n = create_world(length=2, width=2)
    world_grid = set_obstacle(world_grid, startx=8, starty=8, obLen=0.4, obWid=0.4)

    start_point = (16, 4)
    target_point = (4, 16)

    path = []
    safety_path = greedy_search(start_point, target_point, m, n, world_grid, mw=5, hw=1, sw=100, mode='safety')

    path.append((start_point[0], start_point[1], math.pi))

    for i in range(len(safety_path) - 1):
        direction = [safety_path[i+1][0] - safety_path[i][0], safety_path[i+1][1] - safety_path[i][1]]
        if direction == [-1, 0]:
            angle = math.pi
        elif direction == [0, 1]:
            angle = math.pi / 2
        else:
            angle = 3 * math.pi / 4
        
        p = (safety_path[i+1][0], safety_path[i+1][1], angle)
        path.append(p)
    
    return np.array(path)

def generate_speed_path():
    world_grid, m, n = create_world(length=2, width=2)
    world_grid = set_obstacle(world_grid, startx=8, starty=8, obLen=0.4, obWid=0.4)

    start_point = (16, 4)
    target_point = (4, 16)

    speed_path = greedy_search(start_point, target_point, m, n, world_grid, mw=1, hw=2, sw=5, mode='speed')
    
    # Optimization of the generated path
    speed_path = speed_path[:-1]
    speed_path[-1] = target_point

    sim_path = []
    for i in range(len(speed_path)):
        startx, starty = start_point[0], start_point[1]
        curx, cury = speed_path[i][0], speed_path[i][1]

        flag = 0
        for j in range(min(startx, curx), max(startx, curx), 1):
            for k in range(min(starty, cury), max(starty, cury), 1):
                if world_grid[j, k] == 1:
                    flag = 1
                    break

        if flag:
            sim_path.append(start_point)
            start_point = speed_path[i-1]

        if i == len(speed_path) - 1:
            sim_path.append(start_point)

    sim_path.append(target_point)
    
    path = sim_path
    for i in range(1, len(sim_path), 1):
        dy = sim_path[i][1] - sim_path[i-1][1]
        dx = sim_path[i][0] - sim_path[i-1][0]
        angle = math.atan2(dy, dx)
        path[i] = (sim_path[i][0] * 0.1, sim_path[i][1] * 0.1, angle)

    path[0] = (1.6, 0.4, math.pi)
    path[-1] = (0.4, 1.6, math.pi/2)
    return np.array(path)

if __name__ == '__main__':
    # world_grid, m, n = create_world(length=2, width=2)
    # world_grid = set_obstacle(startx=8, starty=8, obLen=0.4, obWid=0.4)

    # start_point = (16, 4)
    # target_point = (4, 16)
    # world_grid[start_point] = 3
    # world_grid[target_point] = 3

    # print(world_grid)
    # path = []
    # came_from, world_grid = Astar_planning(start_point, target_point, m, n, world_grid)
    
    # pre = target_point
    
    # while not equals(pre, start_point):
    #     path.append(pre)
    #     pre = came_from[pre]
    
    safety_path = generate_safety_path()
    print(safety_path)

    speed_path = generate_speed_path()
    print(speed_path)