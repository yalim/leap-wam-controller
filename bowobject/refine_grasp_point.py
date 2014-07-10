import numpy as np
#import cPickle
from os.path import exists
import cv2
import csv

from geometry_msgs.msg import Point
from iri_bow_object_detector.srv import RefineGraspPoint
from iri_perception_msgs.msg import ImagePoint
from get_new_grasp_position import get_new_grasp_position

from bow_detector_utils import ros_ima_to_numpy, save_data_for_log, draw_boxes, BC, persistent_svm, preload_persistent_svm, draw_boxes_2
import rospkg
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path('leap_wam_controller')+'/config/'
CSV_SCORES = PACKAGE_PATH + 'scores.csv'
CSV_POSITIONS_X = PACKAGE_PATH + 'x_pos.csv'
CSV_POSITIONS_Y = PACKAGE_PATH + 'y_pos.csv'
CSV_POSITIONS_Z = PACKAGE_PATH + 'z_pos.csv'
CSV_POSITIONS_UV = PACKAGE_PATH + 'uv.csv'

def cv_select_point(event, x, y, flags, param):
    # param is list where point will be added
    if event and cv2.EVENT_RBUTTONDOWN:
        param.append((x,y))

def user_select_point(cvimage):
    """Select point based on user input."""
    cv2.namedWindow('select_point')
    cv2.imshow('select_point', cvimage)
    selected_point=[]
    cv2.setMouseCallback('select_point', cv_select_point, selected_point)
    while len(selected_point)==0:
        cv2.waitKey(100)
    cv2.destroyWindow('select_point')
    return selected_point


def select_grasp_point(req, ref_method='finddd', svm_file=None, verb=1, basepath='/home/aramisa/experimentos/cloth'):
    P=[(box.point1.x, box.point1.y, box.point2.x, box.point2.y, box.value ) for box in req.posible_solutions]
    P.sort(key=lambda x:x[4], reverse=True)
    if len(P) < 1:
        return Point(0,0,0)
    P0 = P[0]
    image = ros_ima_to_numpy(req.image)

    scores=None #default result

    if ref_method == 'finddd':
        from shogun.Features import RealFeatures, Labels

        svm = preload_persistent_svm(svm_file, 'refine_svm')
        # to delete
        #  if (persistent_svm.refine_svm == None) or (persistent_svm.refine_svm['svm_file'] != svm_file):
        #     print BC.HEADER, "loading refine svm", BC.ENDC
        #     assert svm_file!=None and exists(svm_file)
        #     pf=open(svm_file, 'rb')
        #     svm = cPickle.load(pf)
        #     pf.close()
        #     persistent_svm.refine_svm = {'svm':svm, 'svm_file':svm_file}
        # else:
        #     print BC.HEADER, "Recovering refine persistent_svm", BC.ENDC
        #     svm = persistent_svm.refine_svm['svm_file']

        descs = req.descriptor_set.descriptors
	descs_n = [desc.descriptor for desc in descs if (desc.u > P0[0]) and (desc.u < P0[2]) and (desc.v > P0[1]) and (desc.v < P0[3])]
	xys = [(desc.u, desc.v) for desc in descs if (desc.u > P0[0]) and (desc.u < P0[2]) and (desc.v > P0[1]) and (desc.v < P0[3])]
	x_pos = [desc.point3d.x for desc in descs if (desc.u > P0[0]) and (desc.u < P0[2]) and (desc.v > P0[1]) and (desc.v < P0[3])]
	y_pos = [desc.point3d.y for desc in descs if (desc.u > P0[0]) and (desc.u < P0[2]) and (desc.v > P0[1]) and (desc.v < P0[3])]
	z_pos = [desc.point3d.z for desc in descs if (desc.u > P0[0]) and (desc.u < P0[2]) and (desc.v > P0[1]) and (desc.v < P0[3])]
        descs_n = np.asarray(descs_n) 
        #descriptors should be already normalized! TODO write an assert to check this

	descs_svm=RealFeatures(descs_n.T)
	out=svm.apply(descs_svm)
	if verb>1: print out.get_labels()
	sco=out.get_labels()
        print 'length of sco', len(sco)
        print P0       

        # Save csv files for positions and scores
        if exists(CSV_SCORES):
            fd = open(CSV_SCORES,'a')
            writer2 = csv.writer(fd)
            writer2.writerow(sco)
            fd.close()
        else:
            fd = open(CSV_SCORES,'w')
            writer2 = csv.writer(fd)
            writer2.writerow(sco)
            fd.close()

        if exists(CSV_POSITIONS_X):
            fd = open(CSV_POSITIONS_X,'a')
            writer2 = csv.writer(fd)
            writer2.writerow(x_pos)
            fd.close()
        else:
            fd = open(CSV_POSITIONS_X,'w')
            writer2 = csv.writer(fd)
            writer2.writerow(x_pos)
            fd.close()
        if exists(CSV_POSITIONS_Y):
            fd = open(CSV_POSITIONS_Y,'a')
            writer2 = csv.writer(fd)
            writer2.writerow(y_pos)
            fd.close()
        else:
            fd = open(CSV_POSITIONS_Y,'w')
            writer2 = csv.writer(fd)
            writer2.writerow(y_pos)
            fd.close()
        if exists(CSV_POSITIONS_Z):
            fd = open(CSV_POSITIONS_Z,'a')
            writer2 = csv.writer(fd)
            writer2.writerow(z_pos)
            fd.close()
        else:
            fd = open(CSV_POSITIONS_Z,'w')
            writer2 = csv.writer(fd)
            writer2.writerow(z_pos)
            fd.close()

        if exists(CSV_POSITIONS_UV):
            fd = open(CSV_POSITIONS_UV,'a')
            writer2 = csv.writer(fd)
            writer2.writerow(xys)
            fd.close()
        else:
            fd = open(CSV_POSITIONS_UV,'w')
            writer2 = csv.writer(fd)
            writer2.writerow(xys)
            fd.close()

        X_result, image22, xr, yr, zr, pos2d1, pos2d2 = get_new_grasp_position(sco, xys, z_pos, x_pos, y_pos)
	maxp3d=maxp2d=maxpsco=None
	shape=(480,640) #wtf
        #get canny for overlay
	imabw=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
	#imabw=(255*imabw).astype('uint8') #wtf!!!
        imabw2=cv2.resize(imabw,(shape[1]/3,shape[0]/3),interpolation=cv2.INTER_LINEAR)
	edges=cv2.Canny(imabw2,50,60)
	scores = min(sco)*np.ones((shape[0]/3,shape[1]/3))

	for s,(u,v) in zip(sco,xys):
		if ((u > P0[0]) and (u < P0[2]) and (v > P0[1]) 
                    and (v < P0[3]) and (maxpsco==None or maxpsco<s)):
                    maxp2d=(u,v)
                    maxpsco=s
		u, v = (u/3, v/3)
		scores[v,u]=s
	for d in descs:
		u, v = (d.u/3, d.v/3)
		if (edges[v,u]!=0): scores[v,u]=np.nan
    elif ref_method == 'middle':
	maxp2du = P0[0] + (P0[2] - P0[0])/2
	maxp2dv = P0[1] + (P0[3] - P0[1])/2
	maxp2d = (maxp2du, maxp2dv)
    elif ref_method == 'manual':
        image_select = image.copy()
        draw_boxes(image_select, P)
        selected_point = user_select_point(image_select)
	maxp2d = (selected_point[0][0], selected_point[0][1])

    #draw boxes, save log and show:
    print BC.OKBLUE, 'Res 2D: (' + str(maxp2d[0]) + ',' + str(maxp2d[1]) + ')', BC.ENDC
   # draw_boxes(image, P, maxp2d)
    # Create a new draw_boxes to draw initial and new positions
    print 'New result:' , pos2d1, pos2d2
    draw_boxes_2(image, maxp2d, pos2d1, pos2d2)     
    save_data_for_log(bbox_ima=image, finddd_scores=scores, incr_num=False, fignum=2, log_path=basepath, doshow=False)

    #return
    res = ImagePoint(maxp2d[0], maxp2d[1])
    return res
