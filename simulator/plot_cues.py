import os

import matplotlib.pyplot as plt
import numpy as np
import scipy.signal


def get_labels(file):
    labels = []
    for i, line in enumerate(file):
        labels.append(line.strip().split(','))
    return labels


def get_curves(dir):
    labels = get_labels(open(os.path.join(dir, "labels.txt")))
    all_curves = dict()
    for label in labels:
        curves = np.genfromtxt(os.path.join(dir, label[0] + ".dat"))
        np.place(curves, curves == 0, [curves.max()])  # eliminate zeros (NaN in LIDAR jargon)
        curves = curves.astype(int)
        if curves.ndim < 2:
            curves = np.reshape(curves, [1, len(curves)])
        # smoothing the curve to avoid plenty of local minima in actual LIDAR readings
        all_curves[label[0]] = scipy.signal.savgol_filter(curves, 5, 1)

    return all_curves, labels


all_curves, labels = get_curves('simulator/data')

fig, ax = plt.subplots(1, 1)
fig.suptitle("CURVES", fontsize=14)

curves = all_curves[labels[0][0]]
for label, curves in all_curves.items():
    for curve in curves:
        ax.set_title(label[1])
        ax.set_ylabel("Distance")
        ax.set_xlabel("Angle")
        ax.set_ylim(0, 400)
        ax.set_xlim(0, np.pi / 2)
        plt.plot(np.arange(0, np.pi / 2, np.pi / 2 / 360), curve)

plt.subplots_adjust(left=None, bottom=None, right=None, top=0.8, wspace=0.4, hspace=0.4)

plt.show()
