import json
import numpy as np
import matplotlib.pyplot as plt

from mapgen import generate, Heatmap

SEED = 1
np.random.seed(SEED)
MAX_ITER = 5000
with open("config.json") as config_file:
    config = json.load(config_file)

def plotRoad(segments):
    for segment in segments[:-1]:
        x, y = np.transpose([segment.start, segment.end])
        line_width = 1.5 if segment.highway else 1
        plt.plot(x, y, color=segment, linewidth=line_width)
    plt.axis('scaled')
    plt.show()

def plotGraph(graph, x_lim, y_lim):
    plt.rcParams['axes.facecolor']='#f5f7f8'
    plt.rcParams['savefig.facecolor']='#f5f7f8'

    # Plotting normal streets
    # Plotting contours
    for edge in graph.edges.keys():
        x, y = np.transpose((edge[0], edge[1]))
        if ( graph.edges[edge]["highway"] ):
            continue
        plt.plot(x, y, color="#dadce0", linewidth=3.2)
    for edge in graph.edges.keys():
        x, y = np.transpose((edge[0], edge[1]))
        if ( graph.edges[edge]["highway"] ):
            continue
        x, y = np.transpose((edge[0], edge[1]))
        line_width = 2 if graph.edges[edge]["highway"] else 3
        plt.plot(x, y, color="white", linewidth=2)

    # Plotting highways
    # Plotting contours
    for edge in graph.edges.keys():
        x, y = np.transpose((edge[0], edge[1]))
        if ( not graph.edges[edge]["highway"] ):
            continue
        plt.plot(x, y, color="#f9ad05", linewidth=4)
    for edge in graph.edges.keys():
        x, y = np.transpose((edge[0], edge[1]))
        if ( not graph.edges[edge]["highway"] ):
            continue
        x, y = np.transpose((edge[0], edge[1]))
        plt.plot(x, y, color="#fde293", linewidth=3)

    '''
    for edge in graph.edges.keys():
        x, y = np.transpose((edge[0], edge[1]))
        line_width = 2 if graph.edges[edge]["highway"] else 3
        #plt.plot(x, y, color=graph.edges[edge]["color"], linewidth=line_width)
        line_color = "#fde293" if graph.edges[edge]["highway"] else "#ffffff"
        plt.plot(x, y, color="#adb1ca", linewidth=line_width+.1)
        plt.plot(x, y, color=line_color, linewidth=line_width)
    '''
    plt.axis('scaled')
    plt.ylim(y_lim)
    plt.xlim(x_lim)
    plt.show()

x = 0
y = 0
width = 50000
height = 50000
generator = generate(0, 0, width, height, seed=SEED)
for i in range(MAX_ITER):
    generator.step()
plotGraph(generator.graph, (x, x+width), (y, y+height))