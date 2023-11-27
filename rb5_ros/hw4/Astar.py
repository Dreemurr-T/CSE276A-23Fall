import queue
import numpy as np
from collections import defaultdict

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

def set_obstacle(startx, starty, obLen, obWid):
    l = int(obLen / 0.1) + 1
    w = int(obWid / 0.1) + 1

    world_grid[startx: startx+l, starty:starty+w] = 1

    return world_grid

def heuristic(goal, cur):
    # Manhattan distance on a square grid
    return abs(goal[0] - cur[0]) + abs(goal[1] - cur[1])

def move_cost(next, cur):
    # Move cost from current point to next point measured in square grid
    return abs(next[0] - cur[0]) + abs(next[1] - cur[1])

# def safety_cost():
def equals(goal, cur):
    return goal[0] == cur[0] and goal[1] == cur[1]

def Astar_planning(start, goal, m, n, world_grid):
    directions = [[-1, 0], [0, 1]]
    que = queue.PriorityQueue()
    que.put(start, 0)

    current_cost = defaultdict(float)
    current_cost[start] = 0.0

    path = dict()
    path[start] = None
    visited = set()

    while not que.empty():
        cur = que.get()

        if equals(goal, cur):
            break
        
        for direction in directions:
            nextX, nextY = cur[0] + direction[0], cur[1] + direction[1]

            if nextX < 0 or nextX == m-1 or nextY < 0 or nextY == n-1:
                continue
        
            if world_grid[nextX, nextY] == 1:
                continue

            next = (nextX, nextY)
            new_cost = current_cost[cur] + move_cost(next, cur)

            if next not in current_cost or new_cost < current_cost[next]:
                current_cost[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                que.put(next, priority)
                path[next] = cur
    
    return path


if __name__ == '__main__':
    world_grid, m, n = create_world(length=2, width=2)
    world_grid = set_obstacle(startx=8, starty=8, obLen=0.4, obWid=0.4)

    start_point = (16, 4)
    target_point = (4, 16)
    world_grid[start_point] = 3
    world_grid[target_point] = 3

    path = Astar_planning(start_point, target_point, m, n, world_grid)

    for cur in path:
        next = path[cur]
        if next is not None:
            world_grid[next] = 2
    
    print(world_grid)