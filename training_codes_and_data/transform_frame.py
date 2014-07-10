import numpy as np
from math import sin, cos
def transform_frame(x_list,y_list,z_list):
    '''
    Transformation between two frames given translations and roll, pitch and yaw rotations
    '''
    r = -2.957
    p = 0.053
    y = 1.516
    xt = 0.904
    yt = -0.015
    zt = 0.723

    cr = cos(r)
    sr = sin(r)

    cp = cos(p)
    sp = sin(p)

    cy = cos(y)
    sy = sin(y)

    x_prime = []
    y_prime = []
    z_prime = []

    rpy_matrix = np.matrix([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr + sy*sr, 0],
                            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr - cy*sr, 0],
                            [-sp  ,   cp*sr,        cp*cr,            0],
                            [0,          0,            0,             1]])

    translation_matrix = np.matrix([[1, 0, 0, xt],
                                    [0, 1, 0, yt],
                                    [0, 0, 1, zt],
                                    [0, 0, 0, 1]])

    # for sample in range(len(x_list)):
    #     x_prime_sample = []
    #     y_prime_sample = []
    #     z_prime_sample = []
    #     for index in range(len(x_list[sample])):
    #         point = np.matrix([[x_list[sample][index]],
    #                            [y_list[sample][index]],
    #                            [z_list[sample][index]],
    #                            [1]])
    #         transformed_point = rpy_matrix*translation_matrix*point
    #         x_prime_sample.append(transformed_point[0,0])
    #         y_prime_sample.append(transformed_point[1,0])
    #         z_prime_sample.append(transformed_point[2,0])

    #     x_prime.append(x_prime_sample)
    #     y_prime.append(y_prime_sample)
    #     z_prime.append(z_prime_sample)

    for x_sample, y_sample, z_sample in zip(x_list, y_list, z_list):
        x_prime_sample = []
        y_prime_sample = []
        z_prime_sample = []
        for x, y, z in zip(x_sample, y_sample, z_sample):
            point = np.matrix([[x], [y], [z], [1]])
            transformed_point = translation_matrix*rpy_matrix*point
            x_prime_sample.append(transformed_point[0,0])
            y_prime_sample.append(transformed_point[1,0])
            z_prime_sample.append(transformed_point[2,0])

        x_prime.append(x_prime_sample)
        y_prime.append(y_prime_sample)
        z_prime.append(z_prime_sample)

    return x_prime, y_prime, z_prime

if __name__ == '__main__':
    X,Y,Z = transform_frame([[0],[0]],[[0],[0]],[[0],[0]])
    print X
    print Y
    print Z