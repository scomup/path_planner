import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import pickle


class PolygonEditor:
    def __init__(self, area = [0,0,10,10]):
        self.fig, self.ax = plt.subplots()
        self.patches = []
        self.polygons = []
        self.current_polygon = None
        self.current_points = []
        self.selected_polygon = None
        self.area = area
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.save_button_ax = plt.axes([0.7, 0.0, 0.1, 0.075])
        self.save_button = plt.Button(self.save_button_ax, 'Save')
        self.save_button.on_clicked(self.save_polygons)
        self.load_button_ax = plt.axes([0.8, 0.0, 0.1, 0.075])
        self.load_button = plt.Button(self.load_button_ax, 'Load')
        self.load_button.on_clicked(self.load_polygons)
        self.update_plot()

        plt.show()

    def on_click(self, event):
        if event.inaxes is not None and event.inaxes != self.save_button_ax and event.inaxes != self.load_button_ax:
            if event.button == 1: # left click to add points
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
                if len(self.patches) > 0:
                    for i, patch in enumerate(self.patches):
                        if patch.contains_point((event.xdata, event.ydata)):
                            self.selected_polygon = i
                            self.update_plot()

    def on_key(self, event):
        if event.key == 'd' and self.selected_polygon is not None: # press "D" key to delete polygon
            self.patches.pop(self.selected_polygon)
            self.polygons.pop(self.selected_polygon)
            self.selected_polygon = None
            self.update_plot()

    def save_polygons(self, event=None):
        with open('polygons.pickle', 'wb') as f:
            pickle.dump(self.polygons, f)

    def load_polygons(self, event=None):
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

        collection = PatchCollection(self.patches, alpha=0.4)
        self.ax.add_collection(collection)
        if self.current_points:
            x, y = zip(*self.current_points)
            self.ax.scatter(x, y, color='red', s=5)
        if self.selected_polygon is not None:
            p =self.patches[self.selected_polygon]
            collection = PatchCollection([p], alpha=1)
            self.ax.add_collection(collection)
        plt.draw()


if __name__ == '__main__':
    editor = PolygonEditor()