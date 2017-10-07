import time

from simulator.code.lidar_bot import *
from simulator.code.tk_graphics import Graphics
from simulator.code.wall_map import *


class ciNeuroBotSimulator:
    global global_is_goal_set
    global_is_goal_set = False
    walls = []
    graph = {}
    max_x = 0
    max_y = 0

    """
    A class that encompasses all functionality of the Bot simulator.
    """

    def __init__(self, config_file, goal):
        self.goal = goal
        """
        Initialize the simulator using a configuration file.
        :param config_file: reference to the file with simulation parameters
        """
        self.config = json.load(open(config_file, 'r'))

        # read the specs for the robots
        self.bot_configurators = json.load(open('{}.json'.format(self.config['BOTS']), 'r'))

        self.win = Graphics(self.config['WIN_WIDTH'], self.config['WIN_HEIGHT'])

        # create robots according to the configuration specs
        self.bots = []
        for bot_configuration in self.bot_configurators:
            self.bots.append(
                Bot(
                    goal,
                    self,
                    ai_module=bot_configuration['AI'],
                    position=bot_configuration['START_POS'],
                    direction=bot_configuration['START_DIR'],
                    speed=bot_configuration['SPEED'],
                    radius=bot_configuration['RADIUS'],
                    color=bot_configuration['COLOR'],
                    max_speed=bot_configuration['MAX_SPEED'],
                    max_angle=bot_configuration['MAX_ANGLE'],
                    max_range=bot_configuration['MAX_RANGE'],
                    lidar_resolution=bot_configuration['LIDAR_RESOLUTION'],
                    rays_per_frame=bot_configuration['RAYS_PER_FRAME']
                )
            )

        self.wall_map = WallMap(self.config['MAP'])

    def run(self, visualize=True, number_of_points=1, file=None,
            randomize='normal', randomizeDir='normal', bias=(10, 25, 30)):
        """
        Run the simulator.
        :param visualize: show the animation
        :type visualize: boolean
        :param number_of_points: number of points to take LIDAR scan from
        :type number_of_points: integer
        :param file: file to save the data in
        :type file: string or None
        :param randomize: randomize robot position
        :type randomize: boolean
        :param randomizeDir: randomize robot direction
        :type randomizeDir: boolean
        :param bias: parameters for randomization (along the forward direction, perpendicular, angular)
        :param bias: 3-tuple
        """

        if file != None:
            file = open(file, 'w+')

        # Output Mode
        if visualize:
            while self.win.active and number_of_points > 0:
                start_time = time.perf_counter()

                # update the position of the bots
                for bot in self.bots:
                    bot.update(
                        self.wall_map,
                        self.bots,
                        self,
                        number_of_points=1,
                        visualize=visualize,
                        file=file,
                        randomize=randomize,
                        randomizeDir=randomizeDir,
                        bias=bias)

                # project the new scene onto the GUI
                self.win.animate(self.wall_map, self.bots)

                # control the frequency of GUI refreshes through the FPS parameter
                ifi = 1 / self.config['FPS']  # inter-frame interval
                while time.perf_counter() - start_time < ifi:
                    time.sleep(ifi)
                    # time.sleep(0.3) # for slo-mo
                number_of_points -= 1

            self.win.end()

        # Training Mode
        else:
            self.win.end()

            for bot in self.bots:
                bot.update(
                    self.wall_map,
                    self.bots,
                    self,
                    number_of_points,
                    visualize,
                    file,
                    randomize,
                    randomizeDir,
                    bias)

        if file != None:
            file.close()

    def adjustWalls(self, factor, vector):
        """
        Adjust the maze by scaling it by a factor and then translating by a vector.
        This is useful for mapping physical mazes into simulator pixel-size replicas.
        :param factor: scaling (enlargement) factor
        :param vector: translation (shift) vector
        """
        self.wall_map.scale(factor)
        self.wall_map.translate(vector)
        for bot in self.bots:
            bot.rad = bot.rad * factor
            bot.pos = bot.pos + [x for x in vector]
            bot.speed = bot.speed * factor
            # bot.dir = bot.dir + factor
