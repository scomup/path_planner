import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import pickle

fig, ax = plt.subplots()

patches = []
polygons = []
current_polygon = None
current_points = []
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

def on_click(event):
    global current_polygon, current_points
    if event.inaxes is not None and event.inaxes != save_button_ax and event.inaxes != load_button_ax:
        if event.button == 1: # left click to add points
            if current_polygon is None:
                current_polygon = [event.xdata, event.ydata]
            else:
                current_polygon.extend([event.xdata, event.ydata])
            current_points.append([event.xdata, event.ydata])
            update_plot()
        elif event.button == 3: # right click to close polygon
            if current_polygon is not None:
                current_polygon.extend([current_polygon[0], current_polygon[1]])
                polygon = Polygon(list(zip(current_polygon[::2], current_polygon[1::2])), True)
                patches.append(polygon)
                polygons.append(current_polygon)
                current_polygon = None
                current_points = []
                update_plot()

def save_polygons(event=None):
    global polygons
    with open('polygons.pickle', 'wb') as f:
        pickle.dump(polygons, f)

def load_polygons(event=None):
    global polygons
    with open('polygons.pickle', 'rb') as f:
        polygons = pickle.load(f)
    for polygon in polygons:
        polygon = Polygon(list(zip(polygon[::2], polygon[1::2])), True)
        patches.append(polygon)
    update_plot()

def update_plot():
    global patches, current_points
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    collection = PatchCollection(patches, alpha=0.4)
    ax.add_collection(collection)
    if current_points:
        x, y = zip(*current_points)
        ax.scatter(x, y, color='red', s=5)
    plt.draw()

#def update_plot():
#    ax.clear()
#    for polygon in polygons:
#        ax.add_patch(Polygon([[p.x, p.y] for p in polygon], fill=False))
#        ax.add_line(Line2D([p.x for p in polygon], [p.y for p in polygon], ls='--', c='C0'))
#        for p in polygon:
#            ax.plot(p.x, p.y, 'o', c='C0')
#    ax.set_xlim(0, 10)
#    ax.set_ylim(0, 10)
#    ax.set_aspect('equal')
#    plt.draw()

fig.canvas.mpl_connect('button_press_event', on_click)

save_button_ax = plt.axes([0.7, 0.0, 0.1, 0.075])
save_button = plt.Button(save_button_ax, 'Save')
save_button.on_clicked(save_polygons)
load_button_ax = plt.axes([0.8, 0.0, 0.1, 0.075])
button_load = plt.Button(load_button_ax, 'Load')
button_load.on_clicked(load_polygons)

plt.connect('button_press_event', on_click)
plt.show()

plt.show()