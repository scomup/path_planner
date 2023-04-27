from gui import *
from astar import *


if __name__ == '__main__':
    gui = GUI()
    print("here")
    path_finder = PathFinder()
    gui.set_path_finder(path_finder)
    gui.show()