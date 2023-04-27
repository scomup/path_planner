from gui import *
from astar import *


if __name__ == '__main__':
    print("Use left click to add new point to current polygon.")
    print("Use right click close the polygon.")
    print("Use middle click to select a object.")
    print("Use mouse drag to move a object.")
    print("Push 'd' to delete a selected object.")
    print("Push 'r' to unselect object.")
    gui = GUI()
    path_finder = PathFinder()
    gui.set_path_finder(path_finder)
    gui.show()