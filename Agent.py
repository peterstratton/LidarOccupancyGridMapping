import numpy as np
import vtkplotter as vtk
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
        return vtk.Box((self.x, self.y, self.z), self.width, self.height, self.RISE, size=(), c=self.color, alpha=1)

    def vtk_point_render(self):
        """ Returns a vtk circle representation of the Agent """
        return vtk.Circle(pos=(self.x, self.y, self.z), r=2, fill=True, c='blue')

    def get_endpoint(self, angle, s_range, pos, robot_angle):
        """ Gets the endpoint of the sensor """
        end_x = pos[0] + s_range * math.cos(angle + robot_angle)
        end_y = pos[1] + s_range * math.sin(angle + robot_angle)
        return (end_x, end_y)

    def hitting_grids(self, pixel_map, grid_size, grid_width, width, height, corners):
        """ Determines which grid cells are free or occupied based on the sensor measurement """
        occupied = set()
        free = set()

        # for each "corner" of the line
        for corner in corners:
            row = int(corner[1])
            col = int(corner[0])
            grid_row = math.floor((row  + height/2) / grid_size)
            grid_col = math.floor((col + width/2) / grid_size)
            index = grid_width * grid_row + grid_col
            # map an x,y position to an index in the vtk grid list
            free.add(index)

        # add the last index as occupied
        corner = corners[-1]
        row = int(corner[1])
        col = int(corner[0])
        grid_row = math.floor((row  + height/2) / grid_size)
        grid_col = math.floor((col + width/2) / grid_size)
        index = grid_width * grid_row + grid_col

        free.remove(index)
        occupied.add(index)
        return (occupied, free)

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


    def bresenham(self, map, start, end, scale):
        """ Determines the shortest path of pixels of a line in increments determined by the scale """
        """ Scale of 1 is recommend lol """

        corners = []

        dx = end[0] - start[0]
        dy = end[1] - start[1]

        # handle 0 and negative slopes
        if dx == 0:
            x = start[0]
            y = start[1]
            while y <= end[1]:
                corners.append((x, y))
                y = y + scale
            corners.append((x, y))
        elif dy == 0:
            x = start[0]
            y = start[1]
            while x <= end[0]:
                corners.append((x, y))
                x = x + scale
            corners.append((x, y))
        else:
            # set initial slope
            m = dy/dx

            # determine increment
            inc = 0
            y_inc = None
            if m > 0:
                if m < 1:
                    # Quadrants 1 and 5
                    if dx > 0:
                        # increment y positively
                        inc = scale
                        i = start[1]
                        error_inc = inc

                        # get positive x values
                        x = start[0]
                        values = [x]
                        x = x + scale
                        while x <= end[0]:
                            values.append(x)
                            x = x + scale
                        values.append(end[0])

                        # set if y or x is incremeneted
                        y_inc = True

                        # set adjusted slope
                        m = dy/dx
                    else:
                        # increment y negatively
                        inc = -scale
                        i = start[1]
                        error_inc = -inc

                        # get negative x values
                        x = start[0]
                        values = [x]
                        x = x - scale
                        while x >= end[0]:
                            values.append(x)
                            x = x - scale
                        values.append(end[0])

                        # set if y or x is incremeneted
                        y_inc = True

                        # set adjusted slope
                        m = dy/dx
                else:
                    # Quadrants 2 and 6
                    if dy > 0:
                        # increment x positively
                        inc = scale
                        i = start[0]
                        error_inc = inc

                        # get positive y values
                        y = start[1]
                        values = [y]
                        y = y + scale
                        while y <= end[1]:
                            values.append(y)
                            y = y + scale
                        values.append(end[1])

                        # set if y or x is incremeneted
                        y_inc = False

                        # set adjusted slope
                        m = dx/dy
                    else:
                        # increment x negatively
                        inc = -scale
                        i = start[0]
                        error_inc = -inc

                        # get negative y values
                        y = start[1]
                        values = [y]
                        y = y - scale
                        while y >= end[1]:
                            values.append(y)
                            y = y - scale
                        values.append(end[1])

                        # set if y or x is incremeneted
                        y_inc = False

                        # set adjusted slope
                        m = dx/dy
            else:
                if m > -1:
                    # Quadrants 8 and 4
                    if dx > 0:
                        # increment y negatively
                        inc = -scale
                        i = start[1]
                        error_inc = -inc

                        # get positive x values
                        x = start[0]
                        values = [x]
                        x = x + scale
                        while x <= end[0]:
                            values.append(x)
                            x = x + scale
                        values.append(end[0])

                        # set if y or x is incremeneted
                        y_inc = True

                        # set adjusted slope
                        m = dy/dx
                    else:
                        # increment y positively
                        inc = scale
                        i = start[1]
                        error_inc = -inc

                        # get negative x values
                        x = start[0]
                        values = [x]
                        x = x - scale
                        while x >= end[0]:
                            values.append(x)
                            x = x - scale
                        values.append(end[0])

                        # set if y or x is incremeneted
                        y_inc = True

                        # set adjusted slope
                        m = dy/dx
                else:
                    # Quadrants 7 and 3
                    if dx > 0:
                        # increment x positively
                        inc = scale
                        i = start[0]
                        error_inc = -inc

                        # get negative y values
                        y = start[1]
                        values = [y]
                        y = y - scale
                        while y >= end[1]:
                            values.append(y)
                            y = y - scale
                        values.append(end[1])

                        # set if y or x is incremeneted
                        y_inc = False

                        # set adjusted slope
                        m = dx/dy
                    else:
                        # increment x negatively
                        inc = -scale
                        i = start[0]
                        error_inc = inc

                        # get positive y values
                        y = start[1]
                        values = [y]
                        y = y + scale
                        while y <= end[1]:
                            values.append(y)
                            y = y + scale
                        values.append(end[1])

                        # set if y or x is incremeneted
                        y_inc = False

                        # set adjusted slope
                        m = dx/dy

            error = 0
            for v in values:
                # determine how to set values into the returned list
                if y_inc:
                    corners.append((v, i))
                else:
                    corners.append((i, v))

                # preform bresenham algorithm based on the slope
                if(m > 0):
                    if(error + m < 0.5):
                        error = error + (scale * m)
                    else:
                        i = i + inc
                        error = error + (scale * m) - error_inc
                else:
                    if(error + m > 0.5):
                        error = error + (scale * m)
                    else:
                        i = i + inc
                        error = error + (scale * m) - error_inc

        return corners
