import json
import copy

import numpy as np
from perlin_noise import PerlinNoise

# bounds
X = 0
Y = 0
WIDTH = 500
HEIGHT = 500

with open("config.json") as config_file:
    config = json.load(config_file)

class Heatmap:
    def __init__(self, seed):
        self.noise = PerlinNoise(octaves=3, seed=seed)

    def populationOnRoad(self, road):
        return (self.opulationAt(*road.start) + self.opulationAt(*road.end)) / 2
    
    def opulationAt(self, x, y):
        return self.noise([x/WIDTH, y/HEIGHT])


class Segment:
    def __init__(self, start, end, time_step=0, highway=False, color="black", severed=False, previous_road=None):
        self.start = start
        self.end = end
        self.time_step = time_step
        self.width = config["HIGHWAY_SEGMENT_WIDTH"] if highway else  config["DEFAULT_SEGMENT_WIDTH"]
        self.severed = severed
        self.color = color
        self.highway = highway
        self.length = config["HIGHWAY_SEGMENT_LENGTH"] if highway else config["DEFAULT_SEGMENT_LENGTH"]
        self.previous_road = previous_road

        self.links = {"b": [], "f": []}

    def direction(self):
        aux_vec = (self.end[0]-self.start[0], self.end[1]-self.start[1])
        return -1 * np.sign(np.cross((0,1), aux_vec)) * getAngle((0, 1), aux_vec)

    def linksForEndContaining(self, segment):
        if (segment in self.links["b"]):
            return "b"
        elif (segment in self.links["f"]):
            return "f"
        else:
            return None

    def setupBranchLinks(self):
        for link in self.previous_road.links["f"]:
            self.links["b"].append(link)
            containing_direction = link.linksForEndContaining(self.previous_road)
            if (containing_direction == None):
                return
            link.links[containing_direction].append(self)
        self.previous_road.links["f"].apppend(self)
        self.links["b"].append(previous_road)
        return self.links["b"]
            
        



class Generator:
    def __init__(self, seed):
        self.seed = seed
        self.segments = []
        self.heatmap = Heatmap(seed)

        self.highway_count = 0
        self.queue = np.array(self.makeInitialSegment())
        self._sortQueue()
        return

    # Sort the queue based on the time
    def _sortQueue(self):
        times = [s.time_step for s in self.queue]
        self.queue = self.queue[np.argsort(times)][::-1]

    # A single iteration step
    def step(self):
        try:
            self._sortQueue()
            new_road, self.queue = self.queue[-1], self.queue[:-1] # popping
        except:
            #print("ERROR: queue empty")
            return
        
        accepted = self.localConstraints(new_road)
        if (accepted):
            self.segments.append(new_road)
            for next_road in self.globalGoals(new_road):
                next_road.time_step = new_road.time_step + next_road.time_step + 1
                self.queue = np.append(self.queue, next_road)
        return

    def splitSegment(self, segment, point):
        self.segments.remove(segment)
        new_segment_a = Segment(start=segment.start, end=point, time_step=0, 
                                highway=segment.highway, severed=True, previous_road=segment.previous_road)
        new_segment_b = Segment(start=point, end=segment.end, time_step=0, 
                                highway=segment.highway, severed=True, previous_road=segment.previous_road)
        
        new_segment_a.links["b"] = segment.links["b"]
        new_segment_a.links["f"] = [new_segment_b]
        new_segment_b.links["b"] = [new_segment_a]
        new_segment_b.links["f"] = segment.links["f"]
        self.segments += [new_segment_a, new_segment_b]

        # go through all linked roads at that end, and replace their inverse references from referring to this to referring to the newly created segment
        for link in segment.links["b"]:
            #link.links["f"].remove(segment)
            link.links["b"].append(new_segment_b)
        for link in segment.links["f"]:
            #link.links["b"].remove(segment)
            link.links["f"].append(new_segment_a)

        del(segment)

        return new_segment_a, new_segment_b

    # TO DO
    def localConstraints(self, road):
        def getClosestRoads(road):
            close_roads = []
            start, end = np.array(road.start), np.array(road.end)
            for other in self.segments:
                # Checking distances (can optmize)
                other_start, other_end = np.array(other.start), np.array(other.end)
                min_dist = np.min([np.linalg.norm(start - other_start), np.linalg.norm(end - other_end), 
                                  np.linalg.norm(start - other_end), np.linalg.norm(end - other_start)])
                if (min_dist < config["HIGHWAY_SEGMENT_LENGTH"]):
                    close_roads.append(other)
            return close_roads

        # Check if max number of highways reached
        if (road.highway == True):
            if (self.highway_count == config["HIGHWAY_MAX_COUNT"]):
                print("reached")
                return False
            else:
                self.highway_count += 1
                
        if (config["IGNORE_CONFLICTS"]): return True

        closest_roads = getClosestRoads(road)

        # 1. Checking intersects:
        intersecting_roads = []
        for other in closest_roads:
            intersection = intersects(road, other)
            if (intersection != None and (~np.isclose(intersection, road.start)).sum()!=0 and (~np.isclose(intersection, road.end)).sum()!=0):
                intersecting_roads.append(other)

        if (len(intersecting_roads) > 0):
            idx = np.argmin([distance(intersects(road, r), road.start) for r in intersecting_roads])
            other = intersecting_roads[idx]
            intersection = intersects(road, other)
            angle = np.abs(other.direction() - road.direction()) % 180
            angle = min(angle, 180 - angle)
            if angle < config["MINIMUM_INTERSECTION_DEVIATION"]:
                return False

            # Splitting to create intersection
            #print("intersect:", intersection, f"of {road.start, road.end} AND {other.start, other.end}")
            #print("splitting")
            
            segment_a, segment_b = self.splitSegment(other, intersection)
            road.end = intersection
            road.severed = True
            segment_a.links["f"].append(road)
            segment_b.links["b"].append(road)
            road.links["f"] += [segment_a, segment_b]
            road.color = "red"
            return True

        for other in closest_roads:
            # 2. Checking snap to crossing within radius check
            if(distance(road.end, other.end) <= config["ROAD_SNAP_DISTANCE"]):
                point = other.end
                road.end = point
                road.severed = True

                links = other.links["f"]
                for link in links:
                    if (((~np.isclose(link.start, road.end)).sum==0 and (~np.isclose(link.end, road.start)).sum==0) or
                        ((~np.isclose(link.start, road.start)).sum==0 and (~np.isclose(link.end, road.end)).sum==0)):
                        return False
                
                for link in links:
                    containing = link.linksForEndContaining(other)
                    if (containing == None):
                        return False
                    link.links[containing].append(road)
                    road.links["f"].append(link)

                other.links["f"].append(road)
                road.links["f"].append(other)
                road.color = "blue"
                return True

        for other in closest_roads:
            # 3. Intersection within radius check
            point, distance_segment = distanceToLine(road.end, other)
            if (distance_segment < config["ROAD_SNAP_DISTANCE"]):
                road.end = point
                road.severed = True

                # if intersecting lines are too closely aligned don't continue
                angle = np.abs(other.direction() - road.direction()) % 180
                angle = min(angle, 180 - angle)
                if angle < config["MINIMUM_INTERSECTION_DEVIATION"]:
                    return False
                segment_a, segment_b = self.splitSegment(other, point)
                segment_a.links["f"].append(road)
                segment_b.links["b"].append(road)
                road.links["f"] += [segment_a, segment_b]
                road.color = "green"
                return True

        return True

    # Generate next roads according to the global goals from a build road
    def globalGoals(self, road):
        # Road that continues from current, keeping its properties
        def continueRoad(previous_road, direction):
            return segmentFromDirection(previous_road.end, previous_road.direction() + direction, 
                        highway=previous_road.highway, severed=previous_road.severed, length=previous_road.length)
        # Road that branches from previous, having default properties for normal segment
        def branchRoad(previous_road, direction):
            return segmentFromDirection(previous_road.end, previous_road.direction() + direction, 
                        length=config["DEFAULT_SEGMENT_LENGTH"], time_step = config["NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY"] if previous_road.highway else 0)

        branches = []
        if not road.severed:
            continue_straight = continueRoad(road, 0)
            straight_pop = self.heatmap.populationOnRoad(continue_straight)

            # Extending the highway
            if road.highway:
                max_pop = straight_pop
                best_segment = continue_straight
                for i in range(config["HIGHWAY_POPULATION_SAMPLE_SIZE"]):
                    current_segment = continueRoad(road, randomStraightAngle())
                    current_pop = self.heatmap.populationOnRoad(current_segment)
                    if (current_pop > max_pop):
                        max_pop = current_pop
                        best_segment = current_segment
                branches.append(best_segment)

                if (max_pop > config["HIGHWAY_BRANCH_POPULATION_THRESHOLD"]):
                    if (np.random.random() < config["HIGHWAY_BRANCH_PROBABILITY"]):
                        branches.append(continueRoad(road, -90 + randomBranchAngle()))
                    if (np.random.random() < config["HIGHWAY_BRANCH_PROBABILITY"]):
                        branches.append(continueRoad(road, 90 + randomBranchAngle()))
            elif straight_pop > config["NORMAL_BRANCH_POPULATION_THRESHOLD"]: #or True: # TO DO: check heatmapp
                branches.append(continue_straight)
            
            # Branching to normal streets
            if not config["ONLY_HIGHWAYS"]:
                if (straight_pop > config["NORMAL_BRANCH_POPULATION_THRESHOLD"]): #or True: # TO DO: check heatmapp
                    if (np.random.random() < config["DEFAULT_BRANCH_PROBABILITY"]):
                        branches.append(branchRoad(road, -90 + randomBranchAngle()))
                    if (np.random.random() < config["DEFAULT_BRANCH_PROBABILITY"]):
                        branches.append(branchRoad(road, 90 + randomBranchAngle()))

            
        # Setup links between each current branch and each existing branch stemming from the previous segment   
        for branch in branches:
            branch.previous_road = road
    
        return branches

    # Builds the first one or two network segments
    def makeInitialSegment(self):
        root = Segment((0, 0), (config["HIGHWAY_SEGMENT_LENGTH"], 0), 0, 
                        highway = not config["START_WITH_NORMAL_STREETS"])
        return [root]



def distance(a,b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# Checks if two segments intersect. Return intersection point if it exists
def intersects(segment_a, segment_b):
    line1 = (segment_a.start, segment_a.end)
    line2 = (segment_b.start, segment_b.end)
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    # check if point inside segments
    if (np.isclose(distance(line1[0], line1[1]), distance((x, y), line1[0]) + distance((x, y), line1[1])) and
        np.isclose(distance(line2[0], line2[1]), distance((x, y), line2[0]) + distance((x, y), line2[1]))):
        return (x, y)
    else:
        return None


def distanceToLine(point, segment):
    x, y = point
    p1 = segment.start
    p2 = segment.end

    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2-x1, y2-y1
    det = dx*dx + dy*dy       
    a = ((x-x1)*dx + (y-y1)*dy) / det

    # Making sure points belong to the segment
    a = min(1, max(0, a))
    Px, Py = x1+a*dx, y1+a*dy
    d = distance((Px, Py), (x, y))

    return (Px, Py), d



def segmentFromDirection(start, direction=90, time_step=0, highway=False, color="black", 
                         severed=False, length=config["DEFAULT_SEGMENT_LENGTH"]):
    x = start[0] + length * np.sin((direction * np.pi) / 180)
    y = start[1] + length * np.cos((direction * np.pi) / 180)

    return Segment(start, (x, y), time_step=time_step, highway=highway, color=color, severed=severed)

def getAngle(u, v):
    dot = np.dot(u, v)

    norm_u = np.linalg.norm(u)
    norm_v = np.linalg.norm(v)

    cos_x = dot/(norm_u*norm_v)
    return np.rad2deg(np.arccos(cos_x))

def randomStraightAngle():
    return np.random.uniform(-15, 15)

def randomBranchAngle():
    return np.random.uniform(-3, 3)

def generate(seed):
    return Generator(seed)