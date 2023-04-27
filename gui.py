import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import pickle
import numpy as np

class GUI:
    def __init__(self, area = [0,0,10,10]):
        self.fig, self.ax = plt.subplots()
        self.patches = []
        self.polygons = []
        self.current_polygon = None
        self.current_points = []
        self.selected_polygon = None
        self.selected_point = None
        self.area = area
        self.prev_x = None
        self.prev_y = None
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion) # detect mouse drag events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.path = None
        self.path_finder = None


        self.save_button_ax = plt.axes([0.6, 0.0, 0.12, 0.075])
        self.save_button = plt.Button(self.save_button_ax, 'Save')
        self.save_button.on_clicked(self.save_polygons)
        self.load_button_ax = plt.axes([0.73, 0.0, 0.12, 0.075])
        self.load_button = plt.Button(self.load_button_ax, 'Load')
        self.load_button.on_clicked(self.load_polygons)
        self.path_button_ax = plt.axes([0.86, 0.0, 0.12, 0.075])
        self.path_button = plt.Button(self.path_button_ax, 'FindPath')
        self.path_button.on_clicked(self.find_path)

        start = np.array([0.5, 0.5])
        goal = np.array([9.5, 9.5])
        self.points = []
        self.points.append(start)
        self.points.append(goal)
        self.update_plot()
        
    def show(self):
        plt.show()

    def set_path_finder(self, path_finder):
        self.path_finder = path_finder

    def find_path(self, event):
        if self.path_finder is not None:
            self.path_finder.set_obstacles(self.polygons)
            self.path = self.path_finder.find_path(self.points[0], self.points[1])
            self.update_plot()

    def on_release(self, event):
        if event.button == 1: # left mouse button released
            self.prev_x = None
            self.prev_y = None

    def on_motion(self, event):
        if self.selected_point is not None:
            if event.inaxes is not None and event.button == 1: # left button drag to move start
                self.points[self.selected_point] = np.array([event.xdata, event.ydata])
                self.update_plot()
                return
        if self.selected_polygon is not None:
            if event.inaxes is not None and event.button == 1: # left button drag
                if(self.prev_x is None):
                    self.prev_x = event.xdata
                    self.prev_y = event.ydata
                    return
                dx = event.xdata - self.prev_x
                dy = event.ydata - self.prev_y
                polygon = self.polygons[self.selected_polygon]
                for i in range(0, len(polygon), 2):
                    polygon[i] += dx
                    polygon[i+1] += dy
                self.prev_x = event.xdata
                self.prev_y = event.ydata
                self.update_plot()


    def on_click(self, event):
        if event.inaxes is not None and \
            event.inaxes != self.save_button_ax and \
            event.inaxes != self.load_button_ax and \
            event.inaxes != self.path_button_ax:
            if event.button == 1: # left click to add points
                if self.selected_point is not None or self.selected_polygon is not None:
                    return
                if self.current_polygon is None:
                    self.current_polygon = [event.xdata, event.ydata]
                else:
                    self.current_polygon.extend([event.xdata, event.ydata])
                self.current_points.append([event.xdata, event.ydata])
                self.update_plot()
            elif event.button == 3: # right click to close polygon
                if self.current_polygon is not None:
                    self.current_polygon.extend([self.current_polygon[0], self.current_polygon[1]])
                    polygon = Polygon(list(zip(self.current_polygon[::2], self.current_polygon[1::2])), closed=True)
                    self.patches.append(polygon)
                    self.polygons.append(self.current_polygon)
                    self.current_polygon = None
                    self.current_points = []
                    self.update_plot()
            elif event.button == 2: # middle click to select polygon
                for i, patch in enumerate(self.patches):
                    if patch.contains_point((event.xdata, event.ydata)):
                        self.selected_polygon = i
                        self.selected_point = None
                        self.update_plot()
                        return
                for i, point in enumerate(self.points):
                        if np.linalg.norm(point - np.array([event.xdata, event.ydata])) < 0.5:
                            self.selected_polygon = None
                            self.selected_point = i
                            print(self.selected_point)
                            self.update_plot()
                            return

    def on_key(self, event):
        if event.key == 'd' and self.selected_polygon is not None: # press "D" key to delete polygon
            self.patches.pop(self.selected_polygon)
            self.polygons.pop(self.selected_polygon)
            self.selected_polygon = None
            self.update_plot()
        if event.key == 'r':
            self.selected_polygon = None 
            self.selected_point = None
            self.current_points = []
            self.update_plot()


    def save_polygons(self, event=None):
        with open('polygons.pickle', 'wb') as f:
            pickle.dump(self.polygons, f)

    def load_polygons(self, event=None):
        self.patches = []
        self.polygons = []
        with open('polygons.pickle', 'rb') as f:
            self.polygons = pickle.load(f)
        for polygon in self.polygons:
            patch = Polygon(list(zip(polygon[::2], polygon[1::2])), closed=True)
            self.patches.append(patch)
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(self.area[0], self.area[2])
        self.ax.set_ylim(self.area[1], self.area[3])

        self.ax.scatter(self.points[0][0], self.points[0][1], color='red', s=20)
        self.ax.scatter(self.points[1][0], self.points[1][1], color='blue', s=20)

        self.patches = []
        for polygon in self.polygons:
            patch = Polygon(list(zip(polygon[::2], polygon[1::2])), closed=True)
            self.patches.append(patch)


        collection = PatchCollection(self.patches, alpha=0.4)
        self.ax.add_collection(collection)
        if self.current_points:
            x, y = zip(*self.current_points)
            self.ax.scatter(x, y, color='red', s=5)
        if self.selected_polygon is not None:
            p =self.patches[self.selected_polygon]
            collection = PatchCollection([p], alpha=1)
            self.ax.add_collection(collection)
        if self.selected_point is not None:
            if(self.selected_point == 0):
                self.ax.scatter(self.points[0][0], self.points[0][1], color='red', s=50)
            else:
                self.ax.scatter(self.points[1][0], self.points[1][1], color='blue', s=50)
        if self.path is not None:
                for i in range(len(self.path)-1):
                    self.ax.plot([self.path[i][0], self.path[i+1][0]], [self.path[i][1], self.path[i+1][1]], 'g')


        plt.draw()


if __name__ == '__main__':
    editor = GUI()