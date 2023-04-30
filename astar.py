import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import pickle
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


def load_polygons():
    with open('polygons.pickle', 'rb') as f:
        polygons = pickle.load(f)
    return polygons


class PathFinder:
    def __init__(self):
        self.resolution = 0.1
        self.path = None

    def set_obstacles(self, polygons):
        self.graph = nx.grid_2d_graph(100, 100)
        self.polygons = polygons
        #self._mark_polygons()
        self._modify_graph()

        idx = np.array(self.graph.nodes())
        for i in  idx:
            for p in self.polygons:
                if p.contains_point((i[1] * self.resolution, i[0] * self.resolution)):
                    try:
                        self.graph.remove_node((i[1], i[0]))
                    except:
                        pass

    def _heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def _modify_graph(self):
        for node in self.graph.nodes:
            i, j = node
            neighbors = [(i+1, j), (i-1, j), (i, j+1), (i, j-1), (i+1, j+1), (i-1, j+1), (i+1, j-1), (i-1, j-1)]
            for neighbor in neighbors:
                if neighbor in self.graph:
                    self.graph.add_edge(node, neighbor)
        weights = {(i, j): 1 for i, j in self.graph.edges if abs(i[0] - j[0]) + abs(i[1] - j[1]) == 1}
        weights.update({(i, j): np.sqrt(2) for i, j in self.graph.edges if abs(i[0] - j[0]) + abs(i[1] - j[1]) == 2})
        nx.set_edge_attributes(self.graph, values=weights, name='weight')


    def find_path(self, start, goal):
        self.start = (start / self.resolution).astype(int)
        self.goal = (goal / self.resolution).astype(int)
        try:
            self.path = nx.astar_path(self.graph, tuple(self.start), tuple(self.goal), heuristic=self._heuristic)
        except:
            print("No path!")
            return None

        numpy_array = np.array(self.path) * self.resolution

        return numpy_array 




if __name__ == '__main__':
    polygons = load_polygons()
    start = np.array([0.8, 8.8])
    goal = np.array([9.8, 9.8])
    path_finder = PathFinder()

    path_finder.set_obstacles(polygons)

    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    collection = PatchCollection(path_finder.polygons, alpha=0.4)
    ax.add_collection(collection)

    ax.scatter(start[0], start[1], color='r')
    ax.scatter(goal[0], goal[1], color='b')

    path = path_finder.find_path(start, goal)
    if(path is not None):
        for i in range(len(path)-1):
            ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'g')


    plt.show()  


    #path_finder.plot_map_and_path()
