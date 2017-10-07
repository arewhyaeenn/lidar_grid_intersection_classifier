import matplotlib
import numpy as np

matplotlib.use("TkAgg")
import math
import os
import importlib


def rotate(vec, theta):
    """
    Rotates a 2d vector (2 element numpy array) by theta (radians).
    :param vec: vector to rotate
    :param theta: rotation angle
    :return: rotated vector
    """
    rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
    return rot_mat.dot(vec)


class Bot:
    """
    AI controlled robot which uses lidar to create a 360 degree
    scan of surroundings and navigate.
    """

    path = None

    def __init__(self,
                 goal,
                 sim,
                 ai_module,
                 position,
                 direction,
                 speed=1,
                 radius=15,
                 color='blue',
                 max_speed=1,
                 max_angle=128,
                 max_range=10000,
                 lidar_resolution=180,
                 rays_per_frame=6
                 ):
        """
        Creates a robot with starting position, direction, speed, radius, and color.
        Position and direction should both be 2 element arrays (2D vectors).
        Parameter "ai_module" refers to the name of the .py file with a class called AI to be used by Bot
        to control the movement of the robot.
        """
        self.constants = {
            'MAX_SPEED': max_speed,
            'MAX_ANGLE': np.pi / max_angle,
            'MAX_RANGE': max_range,
            'LIDAR_RESOLUTION': int(lidar_resolution),  # number of readings per one rotation (360 degrees)
            'RAYS_PER_FRAME': int(rays_per_frame)  # number of rays displayed in one frame (180/30 for 30 fps)
        }

        # mod = __import__(ai_module)
        mod = importlib.import_module(ai_module, package='AI')
        self.goal = goal
        self.ai = mod.AI(goal, sim)
        self.color = color
        self.quote = ''

        self.pos = np.array(position)
        dvec = np.array(direction)
        self.dir = dvec  # / np.linalg.norm(dvec)  # normalize direction
        self.speed = np.clip(speed, 0, self.constants['MAX_SPEED'])
        self.rad = radius

        self.img = np.ones(self.constants['LIDAR_RESOLUTION'])
        self.scan_points = [self.pos] * self.constants['RAYS_PER_FRAME']

        self.scan_offset = 0  # for real-time scanning
        self.sim = sim
        self.scale = (-.5, .5)
        self.temp_pos = self.pos
        self.temp_dir = self.dir

    def update(self, wall_map, bots, sim, number_of_points=1,
               visualize=False, file=None,
               randomize=None, randomizeDir=None, bias=(10, 25, 30)):
        """
        Updates the location of the robot applying requested randomization modifications.
        Scans the distances if requested. Saves the data in a file if requested.
        Animates the maze and draws plots for distances if requested.
        
        :param wall_map: the maze as a list of np.arrays that are walls (line segments)
        :param bots: a list of bots in the maze
        :param number_of_points: How many points to generate
        :param visualize: show the animation
        :param file: file to save the data in
        :param randomize: randomize robot position
        :param randomizeDir: randomize robot direction
        :param bias: parameters for randomization (along the forward direction, perpendicular, angular)
        """

        # print("Temp Pos", self.temp_pos)
        # print("Pos", self.pos)
        self.pos = self.temp_pos
        self.dir = self.temp_dir

        # If requested, randomize consistently with the current direction
        if self.dir[0] == 0:
            P = np.random.multivariate_normal(self.pos, [[bias[0], 0], [0, bias[1]]],
                                              number_of_points) if randomize == 'normal' else self.pos + (
                np.random.uniform((-bias[0], -bias[1]), (bias[0], bias[1]),
                                  (number_of_points, 2)) if randomize == 'uniform' else np.zeros((number_of_points, 2)))
        else:
            P = np.random.multivariate_normal(self.pos, [[bias[1], 0], [0, bias[0]]],
                                              number_of_points) if randomize == 'normal' else self.pos + (
                np.random.uniform((-bias[1], -bias[0]), (bias[1], bias[0]),
                                  (number_of_points, 2)) if randomize == 'uniform' else np.zeros((number_of_points, 2)))
        # print('SHAPE: {}\n'.format(P.shape))

        controls = {'angle': 0, 'speed': 0, 'quote': 'Training'}

        for element in P:
            # print('ELEMENT: {}\n'.format(element))

            # Update current postion
            self.pos = np.array(element)

            # Get current position in degrees and add the half of vision chunk
            deg = math.degrees(math.atan2(self.dir[1], self.dir[0]))

            # randomize angle if requested
            if randomizeDir == 'normal':
                # randomly select one possible angle to be scanned upon
                deg = np.random.normal(deg, bias[2], 1).round().astype(np.int)
            elif randomizeDir == 'uniform':
                deg = deg + np.random.randint(-bias[2], +bias[2])
            # print("SELECTED ANGLE: {}\n".format(deg))

            # Grab current x position
            x = math.cos(math.radians(deg))
            # Grab current y position
            y = math.sin(math.radians(deg))
            # Set current direction
            self.dir = np.array([float(x), float(y)])

            # Scan the surroundings from current configuration
            self.scan(wall_map, bots)

            if visualize:
                # Move, maybe delete
                self.move(controls, wall_map, bots)
                # Update gui
                self.sim.win.animate(wall_map, bots)

            if file:
                w_string = ''
                for reading in self.img:
                    w_string += str(reading) + ' '
                file.write(w_string + os.linesep)
                # print('WRITING: {}\n'.format(w_string))

        self.speak(controls)

    def scan(self, wall_map, bots):
        """
        Casts RAYS_PER_FRAME rays around bot, starting from back and moving clockwise,
        determines closest intersection of each ray with walls or bots,
        and sets image elements to lengths of ray segments.
        :param wall_map: the maze as a list of np.arrays that represent line segments (walls)
        :param bots: a list of bots in the maze
        """
        ray_dir = -1 * self.dir  # start scanning from opposite to the current direction of moving
        ray_dir = rotate(ray_dir, self.scan_offset * 2 * np.pi / len(self.img))  # granularity of the scan
        # cast rays
        for i in range(self.scan_offset, self.scan_offset + self.constants['RAYS_PER_FRAME']):
            endpoint = self.pos + ray_dir * self.constants['MAX_RANGE']
            ray = np.array([self.pos, endpoint])
            ray_v = ray[1] - ray[0]
            perp = np.array([-ray_v[1], ray_v[0]])

            # find intersection of ray with all walls
            self.img[i] = self.constants['MAX_RANGE']
            self.scan_points[i % self.constants['RAYS_PER_FRAME']] = self.pos + self.img[i] * ray_dir
            for wall in wall_map.walls:
                # determine if wall lies ahead
                ta = wall[0] - self.pos
                tb = wall[1] - self.pos
                front1 = np.dot(ray_v, ta)
                front2 = np.dot(ray_v, tb)
                side1 = np.dot(perp, ta)
                side2 = np.dot(perp, tb)
                if (front1 < 0 and front2 < 0) or side1 * side2 > 0:
                    continue

                # find point where ray line intersects wall line
                da = ray[1] - ray[0]
                db = wall[1] - wall[0]
                dp = ray[0] - wall[0]
                denom = np.dot(perp, db)
                num = np.dot(perp, dp)
                point = (num / denom) * db + wall[0]

                # determine if point is in wall
                if np.dot(ray_v, point - self.pos) > 0:
                    dist = np.linalg.norm(point - self.pos)
                    # check if it's the closest obstacle
                    if dist < self.img[i]:
                        self.img[i] = dist
                        self.scan_points[i % self.constants['RAYS_PER_FRAME']] = point

            # find intersection of ray with other bots
            for bot in bots:
                if bot == self:
                    continue
                proj_len = (bot.pos - self.pos).dot(ray_v) / np.linalg.norm(ray_v)
                proj_pt = self.pos + proj_len * ray_v / np.linalg.norm(ray_v)
                # self.scan_points[i % self.constants['RAYS_PER_FRAME']] = proj_pt
                # print(proj_len)
                if ray_v.dot(proj_pt - self.pos) < 0:
                    continue
                # check if ray intersects bot
                if np.linalg.norm(proj_pt - bot.pos) < bot.rad:
                    hc_len = (bot.rad ** 2 - (proj_pt - bot.pos).dot(proj_pt - bot.pos)) ** 0.5
                    hc_vec = hc_len * ray_v / np.linalg.norm(ray_v)
                    point1 = proj_pt + hc_vec
                    point2 = proj_pt - hc_vec
                    dist1 = np.linalg.norm(point1 - self.pos)
                    dist2 = np.linalg.norm(point2 - self.pos)
                    # check if it's the closest obstacle
                    if dist1 < self.img[i]:
                        self.img[i] = dist1
                        self.scan_points[i % self.constants['RAYS_PER_FRAME']] = point1
                    if dist2 < self.img[i]:
                        self.img[i] = dist2
                        self.scan_points[i % self.constants['RAYS_PER_FRAME']] = point2

            ray_dir = rotate(ray_dir, 2 * np.pi / len(self.img))
        self.scan_offset = (self.scan_offset + self.constants['RAYS_PER_FRAME']) % self.constants['LIDAR_RESOLUTION']

    def move(self, controls, wall_map, bots):
        """
        Uses controls, which for now will only specify clockwise rotation speed,
        and moves forward unless a collision is detected.
        :param controls: a dictionary with 'angle, 'speed', and 'quote'
        :param wall_map: the maze as a list of np.arrays that represent line segments (walls)
        :param bots: a list of bots in the maze
        """
        # turn
        if 'angle' in controls:
            theta = controls['angle']
            limit = self.constants['MAX_ANGLE']
            theta = np.clip(theta, -limit, limit)
            d = rotate(self.dir, theta)
            self.dir = d  # / np.linalg.norm(d)

        # move forward
        velocity = self.speed * self.dir
        if 'speed' in controls:
            velocity *= np.clip(controls['speed'], 0, self.constants['MAX_SPEED'])
        if not self.collide(velocity, wall_map, bots):
            self.pos = np.add(self.pos, velocity)
        if controls['quote'] == "Done":
            self.pos = np.add(self.ai.graph[self.goal][0], velocity)

    def collide(self, velocity, wall_map, bots):
        """
        Determines if moving ahead will result in collision with a wall or another bot.
        :param velocity: the speed of the bot
        :param wall_map: the maze as a list of np.arrays that represent line segments (walls)
        :param bots: a list of bots in the maze
        :return: False or True
        """

        next_pos = np.add(self.pos, velocity)

        # check collisions with other bots
        for bot in bots:
            if bot == self:
                continue
            dist = np.linalg.norm(bot.pos - next_pos)
            if dist < bot.rad + self.rad:
                return True

        # check collisions with the maze walls
        for wall in wall_map.walls:
            a, b = wall
            seg_len = np.linalg.norm(b - a)
            if seg_len == 0:
                if np.linalg.norm(next_pos - a) < self.rad:
                    return True
                else:
                    continue
            proj_len = np.clip((next_pos - a).dot(b - a) / seg_len ** 2, 0, 1)
            proj_pt = a + proj_len * (b - a)
            if np.linalg.norm(next_pos - proj_pt) < self.rad:
                return True
        # no collision
        return False

    def speak(self, controls):
        if 'quote' in controls:
            self.quote = controls['quote']
