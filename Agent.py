import numpy as np
import vedo as vtk_p
import math


class Agent:
    """ Class Agent acts as the robot in the environment. It preforms movement and sensing. """

    FREE = 0
    OCCUPIED = 1
    RISE = 0

    def __init__(self, x, y, angle):
        """ Initializes the agent """
        self.x = x
        self.y = y
        self.z = 0
        self.angle = angle
        self.width = 1
        self.height = 1
        self.color = 'blue'

    def move(self, pose_info):
        """ Changes the pose (position and angle) of the robot """
        self.x = pose_info[0]
        self.y = pose_info[1]
        self.angle = pose_info[2]

    def vtk_box_render(self):
        """ Returns a vtk box representation of the Agent """
        return vtk_p.Box((self.x, self.y, self.z), self.width, self.height,
                         self.RISE, size=(), c=self.color, alpha=1)

    def vtk_point_render(self):
        """ Returns a vtk circle representation of the Agent """
        return vtk_p.Circle(pos=(self.x, self.y, self.z), r=0.2, c='blue')

    def get_endpoint(self, angle, s_range, pos, robot_angle):
        """ Gets the endpoint of the sensor """
        end_x = pos[0] + s_range * np.cos(angle + robot_angle)
        end_y = pos[1] + s_range * np.sin(angle + robot_angle)
        return (end_x, end_y)

    def length_collide(self, pixel_map, corners):
        """ Returns a new endpoint for the sensor if it hits an occupied pixel """
        # check if any of the vector's "corners" hit an occupied pixel
        for corner in corners:
            row = corner[1]
            col = corner[0]
            if pixel_map[row][col] == self.OCCUPIED:
                # update endpoint of the vector if a collision is detected
                return (col, row)
        return (corners[len(corners) - 1])

    def bresenham(self, start, end):
        """ returns the shortest path through the pixel grid that connects the start and end points """
        points = []
        i_ind = 0
        j_ind = 1
        inc = 1
        swap = False
        reverse = False

        # determine whether to iterate over x or y
        if abs(start[0] - end[0]) < abs(start[1] - end[1]):
            i_ind = 1
            j_ind = 0
            swap = True

        # determine the start and end points
        if start[i_ind] > end[i_ind]:
            temp = end
            end = start
            start = temp
            reverse = True

        # calc deltas
        d_j = end[j_ind] - start[j_ind]
        d_i = end[i_ind] - start[i_ind]

        # determine if j increases or decreases
        if start[j_ind] > end[j_ind]:
            inc = -1
            d_j = abs(d_j)

        m = d_j / d_i  # slope

        j = start[j_ind]
        e = 0
        for i in range(start[i_ind], end[i_ind] + 1):
            if swap:
                points.append((j, i))
            else:
                points.append((i, j))
            if 2 * (e + d_j) < d_i:
                e += d_j
            else:
                j += inc
                e += (d_j - d_i)
        if reverse:
            points.reverse()
        return points
