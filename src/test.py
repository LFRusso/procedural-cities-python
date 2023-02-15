import json
import numpy as np
import matplotlib.pyplot as plt
import shapely.geometry
import descartes
import geopandas as gpd
from matplotlib.patches import Polygon

from mapgen import generate, Heatmap
from plot import plotRoad

SEED = 42
np.random.seed(SEED)
MAX_ITER = 5000
with open("config.json") as config_file:
    config = json.load(config_file)

x = 0
y = 0
width = 10000
height = 10000
generator = generate(0, 0, width, height, seed=SEED)
for i in range(MAX_ITER):
    generator.step()
plotRoad(generator.segments)
#plotGraph(generator.graph, (x, x+width), (y, y+height))