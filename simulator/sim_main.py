from simulator.code.ciNeuroBotSimulator import *

# TODO: add more cues
cfgs = ['simulator/config/maze_xs_config.json']

# runs all configs
for iter in range(len(cfgs)):
    sim = ciNeuroBotSimulator(cfgs[iter], None)
    sim.adjustWalls(5, (50, 50))  # translation of the geometry of a physical maze to simulated GUI
    sim.run(visualize=True,
            number_of_points=200,
            # file='simulator/data/xs.dat',
            randomize='uniform',
            randomizeDir='uniform',
            bias=(20, 40, 15))  # horizontal, vertical, angular
    print('Finished', iter)
