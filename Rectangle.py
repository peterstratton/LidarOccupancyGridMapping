import numpy as np
import vedo as vtk_p


class Rectangle:
    RISE = 0

    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def corners(self):
        corners = [(self.x - self.width, self.y - self.height / 2),
                   (self.x + self.width, self.y - self.height / 2),
                   (self.x - self.width, self.y + self.height / 2),
                   (self.x + self.width, self.y + self.height / 2)]
        return np.array(corners).astype(int)

    def vtk_render(self, color, dotted=False, alpha=1, z_index=0):
        return vtk_p.Box((self.x, self.y, 0), self.width, self.height, self.RISE+z_index, size=(), c=color, alpha=alpha)

    def numpy_render(self, grid):
        for y in range(int(self.y - self.height / 2), int(self.y + self.height / 2 + 1)):
            for x in range(int(self.x - self.width / 2), int(self.x + self.width / 2 + 1)):
                try:
                    grid[y, x] = 1
                except:
                    pass  # out of screen
        return grid
