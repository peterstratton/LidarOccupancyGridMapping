import numpy as np
import vtkplotter as vtk
import math
import Agent
from Agent import *
import Rectangle
from Rectangle import *
import matplotlib.pyplot as plt

# world dimensions
WIDTH = 40
HEIGHT = 40
RISE = 0

# create grid map for occupancy grid mapping
grid_size = 1
num_grid_cells = math.ceil(WIDTH / grid_size) * math.ceil(HEIGHT / grid_size)
grid_width = math.ceil(WIDTH / grid_size)
grid_height = math.ceil(HEIGHT / grid_size)
grid_map = []
log_odd_map = np.zeros(num_grid_cells)
pixel_map = np.zeros((HEIGHT, WIDTH))
confidence = 0.1

# get data
angles = np.load('data/angles.npy')
poses = np.load('data/poses.npy')
ranges = np.load('data/ranges.npy')

# create world
world = vtk.Box([0, 0, 0], WIDTH, HEIGHT, RISE).wireframe()

# create vtk grid to display log odds
for index in range(num_grid_cells):
    grid_r = index // grid_width
    grid_c = index % grid_width

    x = grid_size * grid_c + grid_size / 2 - WIDTH / 2
    y = grid_size * grid_r + grid_size / 2 - HEIGHT /2
    width  = grid_size
    height = grid_size
    grid_map.append(vtk.Box((x, y, 0), width, height, RISE, size=(), c='black', alpha=0.5))

robot = Agent(0, 0, 0)
path = []
index = 0

# for every movement, preform a sense and update length of the sensor
for pose in poses:
    path.append(vtk.Line((robot.x, robot.y, 0), (pose[0], pose[1], 0), c='black', lw=2))
    robot.move(pose)
    # Keeps track of where we are in the program because its long af lol
    if index % 50 == 0:
        print(index)

    sensors = []
    for (angle, s_range) in zip(angles[index], ranges[index]):
        pixels = None
        if not math.isnan(s_range * math.cos(angle + robot.angle)):
            end_x, end_y = robot.get_endpoint(angle, s_range, (robot.x, robot.y), robot.angle)
            endpoint = (end_x, end_y, RISE)

            # get occupied pixels
            pixels = robot.bresenham(pixel_map, (int(robot.x), int(robot.y)), (int(end_x), int(end_y)), 1)
            occupied, free = robot.hitting_grids(pixel_map, grid_size, grid_width, WIDTH, HEIGHT, pixels)
            sensors.append(vtk.Line((robot.x, robot.y, 0), endpoint, c='red', lw=0.5))

        # update log odd values in the occupancy grid
        for i in occupied:
            log_odd_map[i] -= confidence
        for i in free:
            log_odd_map[i] += confidence
        for i in range(num_grid_cells):
            grid_map[i].alpha(1/(math.e ** log_odd_map[i] + 1))

    # render the world every 50 time steps
    index = index + 1
    vtk.show(world, robot.vtk_box_render(), path, sensors, grid_map, axes=1, bg="white", viewup="y", interactive=0)
