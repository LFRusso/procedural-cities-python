import json
import numpy as np
import matplotlib.pyplot as plt

from mapgen import generate, Heatmap

SEED = 1
np.random.seed(SEED)
MAX_ITER = 500
with open("config.json") as config_file:
    config = json.load(config_file)

def plotRoad(segments):
    for segment in segments[:-1]:
        x, y = np.transpose([segment.start, segment.end])
        line_width = 1.5 if segment.highway else 1
        plt.plot(x, y, color=segment.color, linewidth=line_width)
    plt.axis('scaled')
    plt.show()

def plotGraph(graph):
    for edge in graph.edges.keys():
        x, y = np.transpose((edge[0], edge[1]))
        line_width = 1.5 if graph.edges[edge]["highway"] else 1
        plt.plot(x, y, color=graph.edges[edge]["color"], linewidth=line_width)
    plt.axis('scaled')
    plt.show()

generator = generate(SEED)
for i in range(MAX_ITER):
    generator.step()
plotGraph(generator.graph)