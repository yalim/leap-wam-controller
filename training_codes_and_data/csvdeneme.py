import csv
from minimum_distance import find_minimum_distance
from transform_frame import transform_frame
import matplotlib.pyplot as plt
import numpy as np
import re
from image_reconstruct import image_reconstruct
from matplotlib.font_manager import FontProperties

fontP = FontProperties()
fontP.set_size('small')

def real_pose_of_pixel(pixel, uv_list, x_list, y_list, z_list):
    top_left =  map(min, zip(*uv_list))
    real_position1 = (top_left[0] + 3*pixel[0], top_left[1] + 3*pixel[1])
    index = uv_list.index(real_position1)
    return x_list[index], y_list[index], z_list[index]


# Open the csv files
file_x = open('/home/yisleyici/Desktop/wrinkled_csv/config/x_pos.csv','r')
file_y = open('/home/yisleyici/Desktop/wrinkled_csv/config/y_pos.csv','r')
file_z = open('/home/yisleyici/Desktop/wrinkled_csv/config/z_pos.csv','r')
file_scores = open('/home/yisleyici/Desktop/wrinkled_csv/config/scores.csv','r')
file_grasping_position_list = open('/home/yisleyici/Desktop/wrinkled_csv/config/grasping_position_list.csv','r')
file_uv = open('/home/yisleyici/Desktop/wrinkled_csv/config/uv.csv','r')
file_initial_grasping_position_list = open('/home/yisleyici/Desktop/wrinkled_csv/config/initial_grasping_points.csv','r')

reader_x = csv.reader(file_x)
reader_y = csv.reader(file_y)
reader_z = csv.reader(file_z)
reader_scores = csv.reader(file_scores)
reader_grasping_position = csv.reader(file_grasping_position_list)
reader_uv = csv.reader(file_uv)
reader_initial_grasping_position = csv.reader(file_initial_grasping_position_list)

list_x_tf = []
list_y_tf = []
list_z_tf = []
list_scores = []
list_grasping_position = []
list_uv = []
list_initial_grasping_point = []
min_dist_index_list = []
min_dist_initial_index_list = []

# Read the csv files and create the lists as floats
for row in reader_x:
    # print len(row)
    list_x_tf.append([float(row[index]) for index in range(len(row))])

for row in reader_y:
    # print len(row)
    list_y_tf.append([float(row[index]) for index in range(len(row))])

for row in reader_z:
    # print len(row)
    list_z_tf.append([float(row[index]) for index in range(len(row))])

for row in reader_scores:
    # print len(row)
    list_scores.append([float(row[index]) for index in range(len(row))])

for row in reader_grasping_position:
    list_grasping_position.append([float(row[index]) for index in range(len(row))])

for row in reader_uv:
    list_uv.append([(int(re.sub(r'[()]','', row[index]).split(',')[0]), int(re.sub(r'[()]','', row[index]).split(',')[1])) for index in range(len(row))])

for row in reader_initial_grasping_position:
    list_initial_grasping_point.append([float(row[index]) for index in range(len(row))])


list_x_tf = [list_x_tf[0], list_x_tf[3], list_x_tf[4], list_x_tf[6], list_x_tf[7]]
list_y_tf = [list_y_tf[0], list_y_tf[3], list_y_tf[4], list_y_tf[6], list_y_tf[7]]
list_z_tf = [list_z_tf[0], list_z_tf[3], list_z_tf[4], list_z_tf[6], list_z_tf[7]]
list_scores = [list_scores[0], list_scores[3], list_scores[4], list_scores[6], list_scores[7]]
list_uv = [list_uv[0], list_uv[3], list_uv[4], list_uv[6], list_uv[7]]
list_initial_grasping_point = [list_initial_grasping_point[0], list_initial_grasping_point[3], list_initial_grasping_point[4], list_initial_grasping_point[6], list_initial_grasping_point[7]]
list_grasping_position = [list_grasping_position[0], list_grasping_position[2], list_grasping_position[3], list_grasping_position[5], list_grasping_position[6]]

 


list_x, list_y, list_z = transform_frame(list_x_tf, list_y_tf, list_z_tf)

# Find the index of grasu7ped point.
for i in range(len(list_grasping_position)):
    min_dist, index = find_minimum_distance(list_grasping_position[i], list_x[i], list_y[i], list_z[i])
    min_dist_index_list.append(index)

# grasped_position_pixels=[(list_uv[i][j][0],list_uv[i][j][1]) for i, j in enumerate(min_dist_index_list)]
    # print 'distance= ', min_dist, ' index= ', index

print '----------------- O -----------------'

for i in range(len(list_initial_grasping_point)):
    min_dist,index = find_minimum_distance(list_initial_grasping_point[i], list_x[i], list_y[i], list_z[i])
    min_dist_initial_index_list.append(index)
    # print 'distance= ', min_dist, ' index= ', index

max_x_list = []
max_y_list = []
max_z_real_list = []
x_initial_list = []
y_initial_list = []
x_grasped_list = []
y_grasped_list = []
value_list = []
image_list = []
z_grasped = [pose[2] for pose in list_grasping_position]
theta_grasped = [pose[3] for pose in list_grasping_position]

# print z_grasped

for i in range(len(list_uv)):
    aux_z_real_list = []
    image,x,y,x_grasped, y_grasped, x_initial, y_initial, values = image_reconstruct(list_uv[i], list_scores[i],
                                                                       grasped_position_index=min_dist_index_list[i],
                                                                       initial_grasp_position_index=min_dist_initial_index_list[i])
    max_x_list.append(x)
    max_y_list.append(y)

    for j in range(4):
        x_real, y_real, z_real = real_pose_of_pixel((x[j],y[j]), list_uv[i], list_x_tf[i], list_y_tf[i], list_z_tf[i])
        aux_z_real_list.append(z_real)

    max_z_real_list.append(aux_z_real_list)
    x_initial_list.append(x_initial)
    y_initial_list.append(y_initial)
    x_grasped_list.append(x_grasped)
    y_grasped_list.append(y_grasped)
    value_list.append(values)
    image_list.append(image)

    plt.figure()
    plt.imshow(image)
    plt.autoscale(False)
    maximas, = plt.plot(x,y,'ro')
    initial_plt, = plt.plot(x_initial, y_initial, 'bp', ms=10.0)
    grasped_plt, = plt.plot(x_grasped, y_grasped,'ws', ms=10.0)
    plt.legend([grasped_plt, initial_plt, maximas], 
               ['User chosen grasping point','Initial grasping point','Local maximums'], 
               loc=4, 
               numpoints=1,
               prop=fontP)
    plt.colorbar()
    plt.savefig('/home/yisleyici/Desktop/saved_figures/one_button_scores_00'+str(i)+'.png', dpi=200)

print list_scores[-1][2096]
print list_scores[-1][1577]
print list_scores[-1][906]
print list_scores[-1][836]
# print 'Z real: ', max_z_real_list
np.savez('/home/yisleyici/Desktop/feature_space_one_button', 
         max_x_list=max_x_list,
         max_y_list=max_y_list,
         x_initial_list=x_initial_list,
         y_initial_list=y_initial_list,
         x_grasped_list=x_grasped_list,
         y_grasped_list=y_grasped_list,
         value_list=value_list,
         image=image_list,
         uv=list_uv,
         list_x=list_x,
         list_y=list_y,
         list_z=list_z,
         theta_grasped=theta_grasped,
         z_grasped=z_grasped,
         max_z_real_list=max_z_real_list,
         list_scores=list_scores
         )