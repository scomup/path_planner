import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import pickle
import numpy as np
from solver import * 

class GUI:
    def __init__(self, area = [0,-5,10,5]):
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.polygons = []
        self.current_points = None
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
        self.robot_polygon = np.array([[-0.5, 0.2], [0.5, 0.2], [0.5, -0.2], [-0.5, -0.2]])
        self.traj = []


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
        #if self.path_finder is not None:
        #    self.path_finder.set_obstacles(self.polygons)
        #    self.path = self.path_finder.find_path(self.points[0], self.points[1])
        #    self.update_plot()

        solver = Solver()
        n = 20
        dt = 1.
        cur_pose = gtsam.Pose2(0., 0., 0.)
        v = np.array([0.5, 0, 0])
        max_dist = 0.5

        # Add all node.
        for i in range(n):
            solver.add_node(i, cur_pose)
            delta = gtsam.Pose2(v * dt)
            cur_pose = cur_pose * delta

        # Add all odom factor.
        for i in range(n-1):
            solver.add_odom_factor(i, i+1, gtsam.Pose2(v * dt))

        # Add prior pose factor.
        solver.add_prior_factor(0)
        solver.add_prior_factor(n-1)

        # Add all obstacle factor.
        for i in range(n):
            for j in range(len(self.polygons)):
                solver.add_polygon_factor(i, self.polygons[j].get_verts())
        print('start')
        result = solver.solve()
        print('end')    
        self.traj = []
        for i in range(n):
            pose = result.atPose2(i)
            p =  Polygon( transform_polygon( m2v(pose.matrix()), robot_polygon), closed=True)
            self.traj.append(p)
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
                vertices = polygon.get_verts() + np.array([dx, dy])
                polygon.set_xy(vertices)
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
                if self.current_points is None:
                    self.current_points = [[event.xdata, event.ydata]]
                else:
                    self.current_points.append([event.xdata, event.ydata])
                self.update_plot()
            elif event.button == 3: # right click to close polygon
                if self.current_points is not None:
                    polygon = Polygon(np.array(self.current_points), closed=True)
                    self.polygons.append(polygon)
                    self.current_points = None
                    self.update_plot()
            elif event.button == 2: # middle click to select polygon
                for i, patch in enumerate(self.polygons):
                    if patch.contains_point((event.xdata, event.ydata)):
                        self.selected_polygon = i
                        self.selected_point = None
                        self.update_plot()
                        return
                for i, point in enumerate(self.points):
                        if np.linalg.norm(point - np.array([event.xdata, event.ydata])) < 0.5:
                            self.selected_polygon = None
                            self.selected_point = i
                            self.update_plot()
                            return

    def on_key(self, event):
        if (event.key == 'd' or event.key == 'delete' or event.key == 'backspace') \
              and self.selected_polygon is not None: # press "D" key to delete polygon
            self.polygons.pop(self.selected_polygon)
            self.selected_polygon = None
            self.update_plot()
        if event.key == 'r':
            self.selected_polygon = None 
            self.selected_point = None
            self.current_points = None
            self.update_plot()


    def save_polygons(self, event=None):
        with open('polygons.pickle', 'wb') as f:
            pickle.dump(self.polygons, f)

    def load_polygons(self, event=None):
        self.polygons = []
        with open('polygons.pickle', 'rb') as f:
            self.polygons = pickle.load(f)
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(self.area[0], self.area[2])
        self.ax.set_ylim(self.area[1], self.area[3])

        self.ax.scatter(self.points[0][0], self.points[0][1], color='red', s=20)
        self.ax.scatter(self.points[1][0], self.points[1][1], color='blue', s=20)

        collection = PatchCollection(self.polygons, alpha=0.4)
        self.ax.add_collection(collection)
        if self.current_points is not None:
            points = np.array(self.current_points)
            self.ax.scatter(points[:,0], points[:,1], color='red', s=5)
        if self.selected_polygon is not None:
            p =self.polygons[self.selected_polygon]
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
        if self.traj is not []:
            collection = PatchCollection(self.traj, alpha=0.2, edgecolor = 'green', facecolor = 'green')
            self.ax.add_collection(collection)

        plt.draw()

    def save_polygons(self, event=None):
        with open('polygons.pickle', 'wb') as f:
            pickle.dump(self.polygons, f)

    def load_polygons(self, event=None):
        with open('polygons.pickle', 'rb') as f:
            self.polygons = pickle.load(f)
        self.update_plot()



if __name__ == '__main__':
    editor = GUI()
    editor.show()