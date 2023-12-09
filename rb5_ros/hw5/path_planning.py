import numpy as np
import math

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

def equals(goal, cur):
    return goal[0] == cur[0] and goal[1] == cur[1]

def generate_coverage1():
    world_grid, m, n = create_world(length=2, width=2)
    # world_grid = set_obstacle(world_grid, startx=8, starty=8, obLen=0.4, obWid=0.4)

    path = []

    offsetX = 1
    offsetY = 1

    while offsetX <= int(m/2) and offsetY <= int(n/2):
        for i in range(offsetX, m-offsetX-1, 1):
            if world_grid[i, offsetY] != 1:
                path.append((i, offsetY))
        
        for j in range(offsetY, n-offsetY-1, 1):
            if world_grid[m-1-offsetX, j] != 1:
                path.append((m-1-offsetX, j))
        
        for i in range(m-offsetX-1, offsetX, -1):
            if world_grid[i, n-offsetY-1] != 1:
                path.append((i, n-offsetY-1))
        
        for j in range(n-offsetY-1, offsetY, -1):
            if world_grid[offsetX, j] != 1:
                path.append((offsetX, j))
        
        offsetX += 1
        offsetY += 1
    
    if m % 2 == 1 and n % 2 == 1:
        path.append((int(m/2), int(n/2)))
    # waypoints = []

    # for i in range(len(safety_path) - 1):
    #     direction = [safety_path[i+1][0] - safety_path[i][0], safety_path[i+1][1] - safety_path[i][1]]
    #     if direction == [-1, 0]:
    #         angle = math.pi
    #     elif direction == [0, 1]:
    #         angle = math.pi / 2
    #     else:
    #         angle = 3 * math.pi / 4
        
    #     p = (safety_path[i+1][0], safety_path[i+1][1], angle)
    #     path.append(p)
    
    return np.array(path)

def generate_coverage2():
    

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
    
    path = generate_coverage()
    print(path)
    