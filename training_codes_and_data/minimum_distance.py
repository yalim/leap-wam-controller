import math
def find_minimum_distance(point3d, list_x, list_y, list_z):
    '''
    Finds the 3D minimum distance of point3d of all points in list_(x,y,z)
    It returns the index of point and the minimum distance.
    min distance, index = find_minimum_distance(point3d, list_x, list_y, list_z)
    '''
    min_distance = 10000
    for index in range(len(list_x)):
        distance = math.sqrt((point3d[0] - list_x[index])**2 + (point3d[1] - list_y[index])**2 )#+ (point3d[2] - list_z[index])**2)
        if distance <= min_distance:
            min_distance=distance
            min_index = index

    return min_distance, min_index
