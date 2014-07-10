from image_reconstruct import image_reconstruct
import numpy as np
import matplotlib.pyplot as plt
import rospkg
from matplotlib.font_manager import FontProperties
from os import popen
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('leap_wam_controller')

fontP = FontProperties()
fontP.set_size('small')

def get_last_num_log(basestr):
    ps=popen('ls '+basestr+'*.png | cut -d "_" -f2 | cut -d "." -f1 2>/dev/null')
    nums=ps.readlines()
    if len(nums):
        num=max([int(x) for x in nums])
    else:
        num=-1
    return num

def real_pose_of_pixel(pixel, uv_list, x_list, y_list, z_list):
    top_left =  map(min, zip(*uv_list))
    real_position1 = (top_left[0] + 3*int(pixel[0]), top_left[1] + 3*int(pixel[1]))
    print uv_list
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
    print 'Y', Y
    print 'X', X_result
    plt.figure()
    plt.imshow(image)
    plt.autoscale(False)
    maxes_plt, = plt.plot(x_max[:3], y_max[:3], 'ro', ms=10.0)
    initial_plt, = plt.plot(x_max[-1], y_max[-1], 'bp', ms=10.0)
    result_plt, =plt.plot(X_result[0,0],X_result[0,1],'g^', ms = 10.0)
    
    plt.legend([maxes_plt, initial_plt, result_plt],
               ['Maximum points','Initial grasping point','Result from system'],
               loc=4,
               numpoints=1,
               prop=fontP)
    num = get_last_num_log('/home/yisleyici/experimentos/savedscores/im-saved_')
    num += 1
    plt.savefig('/home/yisleyici/experimentos/savedscores/im-saved_%04d.png'%num)
    real_position1 = (top_left[0] + 3*int(X_result[0,0]), top_left[1] + 3*int(X_result[0,1]))
    # xr, yr, zr = real_pose_of_pixel((X_result[0,1],X_result[0,2]), uv, xs, ys, zs)
    xr = None
    yr = None
    zr = None

    return X_result, image, xr, yr, zr, real_position1[0], real_position1[1]

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
