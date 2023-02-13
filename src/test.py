import json
import numpy as np
import matplotlib.pyplot as plt

from mapgen import generate, Heatmap

SEED = 1
np.random.seed(SEED)
MAX_ITER = 1000
with open("config.json") as config_file:
    config = json.load(config_file)

def plotRoad(segments):
    for segment in segments[:-1]:
        x, y = np.transpose([segment.start, segment.end])
        line_width = 1.5 if segment.highway else 1
        plt.plot(x, y, color=segment.color, linewidth=line_width)
    plt.axis('scaled')
    plt.show()


generator = generate(SEED)
for i in range(MAX_ITER):
    generator.step()
    #print(generator.queue)
plotRoad(generator.segments)