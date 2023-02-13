import json
import numpy as np
import matplotlib.pyplot as plt

from mapgen import generate, Heatmap

SEED = 1
np.random.seed(SEED)
#MAX_ITER = 84
MAX_ITER = 10000
#MAX_ITER = 380
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
    #print()
    #for planned in generator.queue:
        #print(planned.highway)
plotRoad(generator.segments)
#generator.step()
#plotRoad(generator.segments)

#print()
#h = Heatmap()
#for segment in generator.segments:
    #print(segment.highway)
    #print(segment.start, segment.end)
    #print(h.populationOnRoad(segment))