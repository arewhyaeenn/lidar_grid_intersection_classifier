import json

import numpy as np

class WallMap:

    """
    A class representing the maze through a list of walls (segments).
    Each wall is an np.array with two endpoints.
    """

    def __init__(self, source):
        """
        Creates a maze as a list of walls from json source.
        Format for each segment of the maze (wall) is [[x1,y1], [x2,y2]].
        Numpy arrays are used for line computations.
        :param source: the name of the file with json data; the file is expected in ../data directory
        """
        wall_list = json.load(open('{}.json'.format(source), 'r'))
        self.walls = []
        for wall in wall_list:
            self.walls.append(np.array(wall))

    def translate(self, vector):
        """
        Adjust the maze by translating all wall endpoints by a vector (point (delat_x, delta_y)).
        This is useful for mapping physical mazes into simulator pixel-size replicas.
        :param vector: a vector to shift the data (endpoints of the walls) by
        """
        # self.walls = list(map(lambda x: x + vector, self.walls))
        self.walls = [w + vector for w in self.walls]

    def scale(self, factor):
        """
        Adjust the maze by scaling it by a factor.
        This is useful for mapping physical mazes into simulator pixel-size replicas.
        :param factor: a factor for scaling
        """
        # self.walls = list(map(lambda x: x * factor, self.walls))
        self.walls = [w * factor for w in self.walls]
