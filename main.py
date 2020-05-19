import numpy as np
import vtkplotter as vtk
import math
import Agent
from Agent import *
import Rectangle
from Rectangle import *

# world dimensions
WIDTH = 400
HEIGHT = 400
RISE = 5

# create grid map for occupancy grid mapping
grid_size = 10
num_grid_cells = math.ceil(WIDTH / grid_size) * math.ceil(HEIGHT / grid_size)
grid_width = math.ceil(WIDTH / grid_size)
grid_height = math.ceil(HEIGHT / grid_size)
grid_map = []
log_odd_map = np.zeros(num_grid_cells)
pixel_map = np.zeros((HEIGHT, WIDTH))
confidence = 0.01

# get data
angles = np.load('data/angles.npy')
poses = np.load('data/poses.npy')
for pose in poses:
    pose[0] = pose[0] * 10
    pose[1] = pose[1] * 10

ranges = np.load('data/ranges.npy')
ranges = ranges * 10

# create vtk objects
world = vtk.Box([0, 0, 0], WIDTH, HEIGHT, RISE).wireframe()
left_boundary = Rectangle(5 - WIDTH / 2, 0, 10, HEIGHT)
right_boundary = Rectangle(WIDTH / 2 - 5, 0, 10, HEIGHT)
bottom_boundary = Rectangle(0, -HEIGHT/2 + 5, WIDTH, 10)
top_boundary = Rectangle(0, HEIGHT/2 - 5, WIDTH, 10)
left_obstacle = Rectangle(-WIDTH/4, HEIGHT/4, 100, 100)
right_obstacle = Rectangle(WIDTH/4, -HEIGHT/4, 100, 100)

# render boundaries and obstacles
boundaries = [left_boundary.vtk_render('black'), right_boundary.vtk_render('black'), bottom_boundary.vtk_render('black'), top_boundary.vtk_render('black')]
pixel_map = left_obstacle.numpy_render(pixel_map)
pixel_map = right_obstacle.numpy_render(pixel_map)


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
    sensors = []
    for (angle, s_range) in zip(angles[index], ranges[index]):
        if not math.isnan(s_range * math.cos(angle + robot.angle)):
            end_x, end_y = robot.get_endpoint(angle, s_range, (robot.x, robot.y), robot.angle)

            corners = robot.bresenham(pixel_map, (int(robot.x), int(robot.y)), (int(end_x), int(end_y)), 1)
            occupied, free = robot.hitting_grids(pixel_map, grid_size, grid_width, WIDTH, HEIGHT, corners)
            new_endpoint = robot.length_collide(pixel_map, corners)

            new_endpoint = (new_endpoint[0], new_endpoint[1], 0)
            sensors.append(vtk.Line((robot.x, robot.y, 0), new_endpoint, c='red', lw=2))

        # update log odd values in the occupancy grid
        for i in occupied:
            log_odd_map[i] -= confidence
        for i in free:
            log_odd_map[i] += confidence
        for i in range(num_grid_cells):
            grid_map[i].alpha(1/(math.e ** log_odd_map[i] + 1))

    # move robot and increment iteration counter
    robot.move(pose)
    index = index + 1
    # render the world
    vtk.show(world, robot.vtk_render(), path, sensors, boundaries, grid_map, axes=1, bg="white", viewup="y", interactive=0)
