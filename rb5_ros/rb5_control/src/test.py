import numpy as np

def waypoint_reader(file_path):
    waypoints = np.loadtxt(file_path, delimiter=',')

    return waypoints


if __name__ == '__main__':
    waypoint_reader = waypoint_reader('rb5_control/src/waypoints.txt').tolist()
    
    