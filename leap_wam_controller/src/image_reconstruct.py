from scipy.ndimage.filters import maximum_filter, minimum_filter
from scipy.ndimage.morphology import generate_binary_structure, binary_erosion
import scipy.ndimage as ndimage
import numpy as np


def image_reconstruct(uv, scores, find_max_min = True, grasped_position_index = None, initial_grasp_position_index = None):
    bottom_right = map(max, zip(*uv))
    top_left =  map(min, zip(*uv))

    # print 'bottom_right', bottom_right
    # print 'top_left', top_left

    image = np.empty((((bottom_right[1] - top_left[1])/3+1), (bottom_right[0] - top_left[0])/3+1))
    image[:] = np.NAN
    for s,(u,v) in zip(scores,uv):
        u, v = ((u - top_left[0])/3, (v - top_left[1])/3)
        image[v,u] = s
    
    if find_max_min:
        neighborhood_size = 10
        threshold = 0.0 #0025
        image = ndimage.gaussian_filter(image, sigma = 2, mode='constant')
        data_max = maximum_filter(image, neighborhood_size)
        maxima = (image == data_max)
        data_min = minimum_filter(image, neighborhood_size)
        diff = ((data_max - data_min) > threshold)
        maxima[diff == 0] = 0

        labeled, num_objects = ndimage.label(maxima)
        slices = ndimage.find_objects(labeled)
        x, y = [], []
        for dy,dx in slices:
            x_center = (dx.start + dx.stop - 1)/2
            x.append(x_center)
            y_center = (dy.start + dy.stop - 1)/2    
            y.append(y_center)

        # neighborhood = generate_binary_structure(2,2)
        # local_max = maximum_filter(image, footprint=neighborhood)==image
        # print local_max
        # background = (image ==0)
        # eroded_background = binary_erosion(background, structure=neighborhood, border_value=1)
        # detected_peaks = local_max - eroded_background


        values = image[y,x]
        zipped = zip(values,x,y)
        zipped.sort(key = lambda t: t[0])
        unzipped = [list(t) for t in zip(*zipped[-4:])]

        # return image, unzipped[1], unzipped[2]

    if grasped_position_index is not None:
        x_grasped, y_grasped = uv[grasped_position_index]
        x_grasped, y_grasped = ((x_grasped - top_left[0])/3, (y_grasped - top_left[1])/3)
        # print x_grasped, y_grasped

        x_initial, y_initial = uv[initial_grasp_position_index] 
        x_initial, y_initial = ((x_initial - top_left[0])/3, (y_initial - top_left[1])/3)
        # print x_initial, y_initial
        return image, unzipped[1], unzipped[2], x_grasped, y_grasped, x_initial, y_initial, unzipped[0]

    return image, unzipped[1], unzipped[2], unzipped[0]
