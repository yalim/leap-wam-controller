from image_reconstruct import image_reconstruct
import numpy as np
import matplotlib.pyplot as plt
import rospkg
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('leap_wam_controller')

def real_pose_of_pixel(pixel, uv_list, x_list, y_list, z_list):
    top_left =  map(min, zip(*uv_list))
    real_position1 = (top_left[0] + 3*int(pixel[0]), top_left[1] + 3*int(pixel[1]))
    index = uv_list.index(real_position1)
    return x_list[index], y_list[index], z_list[index]

def get_new_grasp_position(scores, uv, zs, xs=None, ys=None):
    image, x_max, y_max, values = image_reconstruct(uv, scores)
    z = []
    score = []
    npzfile_A = np.load(PACKAGE_PATH+'/config/train_matrix.npz')

    A = npzfile_A['A']
    # Find index of maximums
    top_left =  map(min, zip(*uv))
    for x, y in zip(x_max, y_max):
        real_position = (top_left[0] + 3*x, top_left[1] + 3*y)
        index = uv.index(real_position)
        z.append(zs[index])
        score.append(scores[index])
    Y = np.array((x_max,y_max,z,values))
    Y = np.reshape(Y,(1,16))
    X_result = np.dot(Y, A)

    xr, yr, zr = real_pose_of_pixel((X_result[0,1],X_result[0,2]), uv, xs, ys, zs)

    return X_result, image, xr, yr, zr, real_position[0], real_position[1]

if __name__ == '__main__':
    npzfile = np.load(PACKAGE_PATH+'/config/feature_space.npz')

    uv_list = npzfile['uv']
    value_list = npzfile['list_scores']
    list_x = npzfile['list_x']
    list_y = npzfile['list_y']
    list_z = npzfile['list_z']
    X_result,image, xr, yr, zr, pos2d1, pos2d2 = get_new_grasp_position(value_list[-1], uv_list[-1], list_z[-1])
    # xr, yr, zr = real_pose_of_pixel((X_result[0,1],X_result[0,2]), 
    #                                 uv_list.flatten().tolist()[-1], 
    #                                 list_x[-1], list_y[-1], list_z[-1])
    print X_result
    print xr, yr, zr
    plt.imshow(image)
    plt.show()