import numpy as np
import matplotlib.pyplot as plt
import pickle


def closest_point_on_line_segment_2d(point, line_start, line_end):
    diff = line_end - line_start
    sq_norm = np.sum(diff**2)
    
    if sq_norm == 0:
        return line_start

    u = ((point[0] - line_start[0]) * diff[0] + (point[1] - line_start[1])*diff[1]) / sq_norm
    
    if u <= 0: return line_start
    elif u >= 1: return line_end
    
    return line_start + u*diff

def distance_point_to_segment_2d(point, line_start, line_end):
    return np.linalg.norm(point - closest_point_on_line_segment_2d(point, line_start, line_end))

def check_line_segments_intersection_2d(line1_start, line1_end, line2_start, line2_end):
    line1 = line1_end - line1_start
    line2 = line2_end - line2_start
  
    denom = line1[0] * line2[1] - line2[0] * line1[1]
    if (denom == 0):
        return False
    denomPositive = denom > 0

    aux = line1_start - line2_start
  
    s_numer = line1[0] * aux[1] - line1[1] * aux[0]
    if ((s_numer < 0) == denomPositive):
        return False # No collision

    t_numer = line2[0] * aux[1] - line2[1] * aux[0]
    if ((t_numer < 0) == denomPositive):
        return False # No collision

    if (((s_numer > denom) == denomPositive) or ((t_numer > denom) == denomPositive)):
        return False # No collision
  
    # Otherwise collision detected
    t = t_numer / denom
  
    intersection = line1_start + t * line1;

    return True

def distance_segment_to_segment_2d(line1_start, line1_end, line2_start, line2_end):
    # check if segments intersect
    if check_line_segments_intersection_2d(line1_start, line1_end, line2_start, line2_end):
        return 0
    
    # check all 4 combinations
    distances = []
    
    distances.append(distance_point_to_segment_2d(line1_start, line2_start, line2_end))
    distances.append(distance_point_to_segment_2d(line1_end, line2_start, line2_end))
    distances.append(distance_point_to_segment_2d(line2_start, line1_start, line1_end))
    distances.append(distance_point_to_segment_2d(line2_end, line1_start, line1_end))
    
    return min(distances)


def distance_point_to_polygon_2d(point, vertices):
    dist = np.inf
    
    # the polygon is a point
    if vertices.shape[0] == 1:
        return np.linalg.norm(point - vertices[0])
    
    # check each polygon edge
    for i in range(vertices.shape[0]-1):
        new_dist = distance_point_to_segment_2d(point, vertices[i], vertices[i+1])
        if new_dist < dist:
            dist = new_dist
    
    if vertices.shape[0] > 2: # if not a line close polygon
        new_dist = distance_point_to_segment_2d(point, vertices[-1], vertices[0]) # check last edge
        if new_dist < dist:
            return new_dist
    
    return dist


def distance_segment_to_polygon_2d(line_start, line_end, vertices):
    dist = np.inf
    
    # the polygon is a point
    if (vertices.shape[0] == 1):
        return distance_point_to_segment_2d(vertices[0], line_start, line_end)
    
    size = vertices.shape[0]
    # check each polygon edge
    for i in range(vertices.shape[0]):
        new_dist = distance_segment_to_segment_2d(line_start, line_end, vertices[i], vertices[(i+1)%size])
        if (new_dist < dist):
            dist = new_dist

    return dist


def distance_polygon_to_polygon_2d(vertices1, vertices2):
    dist = np.inf
    
    # the polygon1 is a point
    if len(vertices1) == 1:
        return distance_point_to_polygon_2d(vertices1[0], vertices2)
    
    # check each edge of polygon1
    for i in range(len(vertices1)-1):
        new_dist = distance_segment_to_polygon_2d(vertices1[i], vertices1[i+1], vertices2)
        if new_dist < dist:
            dist = new_dist
    
    if len(vertices1) > 2: # if not a line close polygon1
        new_dist = distance_segment_to_polygon_2d(vertices1[-1], vertices1[0], vertices2) # check last edge
        if new_dist < dist:
            return new_dist
    
    return dist


with open('polygons.pickle', 'rb') as f:
    polygons = pickle.load(f)


distance = distance_polygon_to_polygon_2d(polygons[0].get_verts(), polygons[1].get_verts())
print(distance)


#x_values1 = [0, 0.2]
#y_values1 = [0, 0.2]
#
#x_values2 = [0, 1]
#y_values2 = [1, 0]
#
#plt.plot(x_values1, y_values1)
#plt.plot(x_values2, y_values2)
#
#plt.show()
