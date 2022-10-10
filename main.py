import numpy as np
import vedo as vtk_p
import math
import Agent
from Agent import *
import Rectangle
from Rectangle import *
import matplotlib.pyplot as plt

# confidence to use when calculating log odds
FREE_CONFIDENCE = 0.3
OCCUPIED_CONFIDENCE = 0.9

# world dimensions
WIDTH = None
HEIGHT = None
RISE = 0
GRID_SIZE = 0.2
EXTEND_AREA = 1


def inverse_sensor_model(occupied):
    if occupied:
        return math.log(OCCUPIED_CONFIDENCE / (1 - OCCUPIED_CONFIDENCE))
    else:
        return math.log(FREE_CONFIDENCE / (1 - FREE_CONFIDENCE))


def plot(xs, ys):
    colors = (0, 0, 0)
    area = np.pi*3

    # Plot
    plt.scatter(np.asarray(xs), np.asarray(ys), s=area, c=colors, alpha=0.5)
    plt.title('Scatter Plot of Lidar Endpoints')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()


def main():
    plot = vtk_p.Plotter(interactive=1)

    # get data
    angles = np.load('data/angles.npy')
    poses = np.load('data/poses.npy')
    ranges = np.load('data/ranges.npy')

    # Calculate world coordinates and dimensions
    robot_orientation = np.array(
        [[poses[i, -1] for _ in range(angles.shape[1])] for i in range(poses.shape[0])])
    robot_coordinate_x = np.array(
        [[poses[i, 0] for _ in range(angles.shape[1])] for i in range(poses.shape[0])])
    robot_coordinate_y = np.array(
        [[poses[i, 1] for _ in range(angles.shape[1])] for i in range(poses.shape[0])])

    ox = np.cos(angles + robot_orientation) * ranges + robot_coordinate_x
    oy = np.sin(angles + robot_orientation) * ranges + robot_coordinate_y

    minx = math.floor(min(ox.flatten()) - EXTEND_AREA / 2.0)
    miny = math.floor(min(oy.flatten()) - EXTEND_AREA / 2.0)
    maxx = math.ceil(max(ox.flatten()) + EXTEND_AREA / 2.0)
    maxy = math.ceil(max(oy.flatten()) + EXTEND_AREA / 2.0)

    HEIGHT = maxy - miny
    WIDTH = maxx - minx

    grid_width = int(round((maxx - minx) / GRID_SIZE))
    grid_height = int(round((maxy - miny) / GRID_SIZE))
    num_grid_cells = grid_width * grid_height

    # create world
    world = vtk_p.Box([(WIDTH)/2+minx, (HEIGHT)/2+miny, 0],
                      WIDTH, HEIGHT, 0).wireframe()

    # initialize log odds and grid maps
    log_odd_map = np.zeros(num_grid_cells)

    # initialize vtk grid objects
    grid_map = []
    index = 0
    for j in range(grid_height):
        for i in range(grid_width):
            x = i * GRID_SIZE + minx
            y = j * GRID_SIZE + miny
            index += 1
            grid_map.append(vtk_p.Box((x, y, 0), GRID_SIZE,
                            GRID_SIZE, RISE, size=(), c='black', alpha=0.5))

    # initialize agent object
    robot = Agent(0, 0, 0)
    path = []
    index = 0

    xs = []
    ys = []

    # for every movement, preform a sense and update length of the sensor
    for pose in poses:
        path.append(vtk_p.Line((robot.x, robot.y, 0),
                    (pose[0], pose[1], 0), c='black', lw=2))
        robot.move(pose)

        s_index = 0
        sensors = []
        # determine if spaces in the grid are occupied or free based on the sensor and range measurements
        for (angle, s_range) in zip(angles[index], ranges[index]):
            pixels = None
            if not math.isnan(s_range * math.cos(angle + robot.angle)):
                # get sensor endpoint
                end_x, end_y = robot.get_endpoint(
                    angle, s_range, (robot.x, robot.y), robot.angle)
                endpoint = (end_x, end_y, RISE)
                end_x = int(round((end_x - minx) / GRID_SIZE))
                end_y = int(round((end_y - miny) / GRID_SIZE))

                # translate the robot's position to grid indicies
                start_x = int(round((robot.x - minx) / GRID_SIZE))
                start_y = int(round((robot.y - miny) / GRID_SIZE))

                # add endpoint to plot
                xs.append(end_x)
                ys.append(end_y)

                # get occupied pixels
                pixels = robot.bresenham((start_x, start_y), (end_x, end_y))

                # display every fifth sensor
                if s_index % 5 == 0:
                    sensors.append(
                        vtk_p.Line((robot.x, robot.y, RISE), endpoint, c='red', lw=0.5))
                s_index = s_index + 1

                # update log odd for free cells
                for i in range(len(pixels) - 1):
                    idx = int(pixels[i][0] + pixels[i][1] * grid_width)
                    log_odd_map[idx] += inverse_sensor_model(False)
                    grid_map[idx].alpha(1 - 1/(math.e ** log_odd_map[idx] + 1))

                # update log odds for occupied cells
                idx = int(pixels[len(pixels) - 1][1]
                          * grid_width + pixels[len(pixels) - 1][0])
                log_odd_map[idx] += inverse_sensor_model(True)
                grid_map[idx].alpha(1 - 1/(math.e ** log_odd_map[idx] + 1))

        # init plotter
        if index == 0:
            plot.show(world, robot.vtk_point_render(),
                      path, sensors, grid_map, interactive=True)

        # render the world every 50 time steps
        if index % 100 == 0:
            print(index)
            plot.remove(plot.actors)
            plot.add(world, robot.vtk_point_render(), path, sensors, grid_map)

        index = index + 1
    plot(xs, ys)


if __name__ == "__main__":
    main()
