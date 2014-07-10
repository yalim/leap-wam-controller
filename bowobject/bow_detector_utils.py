import roslib; roslib.load_manifest('iri_bow_object_detector')

import numpy as np
import cv2
from matplotlib import pyplot
from cv_bridge import CvBridge, CvBridgeError
from os import system, getenv, popen
from os.path import exists
import cPickle

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''
BC = bcolors()
#-----------------------------------------------------

#persistent global variables
class SvmContainer:
    def __init__(self):
        self.p = {}
        self.p['refine_svm']=None
        self.p['non_linear_svm']=None
        self.p['linear_svm']=None

persistent_svm = SvmContainer()
#----------------------------------------------------------

def preload_persistent_svm(svm_file, svm_type):
    if (persistent_svm.p[svm_type] == None) or (persistent_svm.p[svm_type]['svm_file'] != svm_file):
        print BC.HEADER, "loading", svm_type, BC.ENDC
        assert svm_file!=None and exists(svm_file)
        if svm_type in ['non_linear_svm','refine_svm'] :
            pf=open(svm_file, 'rb')
            svm = cPickle.load(pf)
            if svm_type == 'non_linear_svm':
                chi2_width = cPickle.load(pf)
                AA = cPickle.load(pf)
                BB = cPickle.load(pf)
                pf.close()
                persistent_svm.p['non_linear_svm'] = {'svm':svm, 'chi2_width':chi2_width, 'A':AA, 'B':BB, 'svm_file':svm_file}
            else:
                persistent_svm.p['refine_svm'] = {'svm':svm, 'svm_file':svm_file}
            pf.close()
        elif svm_type == 'linear_svm':
            persistent_svm.p['linear_svm']  = {'w':np.load(svm_file), 'svm_file':svm_file}

    #return appropriate svm
    if svm_type == 'linear_svm':
        return persistent_svm.p['linear_svm']['w']
    if svm_type == 'non_linear_svm':
        return (persistent_svm.p['non_linear_svm']['svm'],
                persistent_svm.p['non_linear_svm']['chi2_width'],
                persistent_svm.p['non_linear_svm']['A'],
                persistent_svm.p['non_linear_svm']['B'])
    if svm_type == 'refine_svm':
        return persistent_svm.p['refine_svm']['svm']




def opencv_image_as_array(im):
  """Interface image from OpenCV's native format to a numpy array.

  note: this is a slicing trick, and modifying the output array will also change
  the OpenCV image data.  if you want a copy, use .copy() method on the array!
  """
  import numpy as np
  w, h, n = im.width, im.height, im.channels
  modes = {1:"L", 3:"RGB"}#, 4:"RGBA"}
  if n not in modes:
    raise StandardError('unsupported number of channels: {0}'.format(n))
  out = np.asarray(im) if n == 1 else np.asarray(im)[:,:,::-1]  ## BGR -> RGB
  return out
#----------------------------------------------------

def ros_ima_to_numpy(ros_ima):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv(ros_ima, "rgb8")
    except CvBridgeError, e:
        print e
    return opencv_image_as_array(cv_image).copy()
#----------------------------------------------------

def save_extra_training_data(image, mask, path_pcd='/tmp/tmp_cloud.pcd', save_dir='experimentos/cloths/newtrain/'):
#User to generate BB data ----------------------------------
    home_path = getenv("HOME")
    if save_dir[0]!='/':
        basestr_dir = home_path + save_dir
    else:
        basestr_dir= save_dir
    
    #get current id num
    if exists(basestr_dir+'/allgt.txt'):
        pf = open(basestr_dir+'/allgt.txt')
        num = int([x for x in pf.readlines() if x.split()!=[]][-1].split()[0][1:]) +1
        pf.close()
    else:
        num = 1

    cv2.imwrite(basestr_dir+'/masks/t%05d.png'%num, mask.astype('uint8'))
    cv2.imwrite(basestr_dir+'/png/t%05d.png'%num, image)
    system('mv '+path_pcd+' '+basestr_dir+'/pcd/t%05d.pcd'%num) # assuming pcd will be saved inside cpp code!
    system('gzip '+basestr_dir+'/pcd/t%05d.pcd'%num)
    #get gt bbox
    fig = pyplot.figure(143)
    pyplot.title('INDICATE TOP-LEFT AND BOTTOM-RIGHT COORDINATES OF COLLAR')
    pyplot.imshow(cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
    pyplot.draw()
    pyplot.draw()
    XY_gt_bbox = pyplot.ginput(2,timeout=60) # [(x,y), (x,y), ...]
    pyplot.close(fig)
    pf = open(basestr_dir+'/allgt.txt', 'a')
    pf.write('\nt%05d '%num+' '.join([str(x[0])+' '+str(x[1]) for x in XY_gt_bbox]))
    pf.close()
    return XY_gt_bbox
#----------------------------------------

def get_last_num_log(basestr):
    ps=popen('ls '+basestr+'*.png | cut -d "_" -f2 | cut -d "." -f1 2>/dev/null')
    nums=ps.readlines()
    if len(nums):
        num=max([int(x) for x in nums])
    else:
        num=-1
    return num

def draw_boxes_2(image, maxp2d, s2dpos1, s2dpos2):
    x,y = maxp2d
    cv2.rectangle(image,(x-5,y-1), (x+5, y+1), (255,255,255), 2)
    cv2.rectangle(image,(x-1,y-5), (x+1, y+5), (255,255,255), 2)
    cv2.rectangle(image,(x-1,y-1), (x+1, y+1), (0,0,0), 1)

    cv2.rectangle(image,(s2dpos1-5,s2dpos2-1), (s2dpos1+5, s2dpos2+1), (0,0,0), 2)
    cv2.rectangle(image,(s2dpos1-1,s2dpos2-5), (s2dpos1+1, s2dpos2+5), (0,0,0), 2)
    cv2.rectangle(image,(s2dpos1-1,s2dpos2-1), (s2dpos1+1, s2dpos2+1), (255,255,255), 1)

def draw_boxes(image, P, maxp2d=None):
    probs = [wi[4] for wi in P] #get probs
    maxprob = max(probs)
    i_maxprob = np.argmax(probs)
    minprob = min(probs)
    for ii,wi in enumerate(P):
        if minprob==maxprob:
            wi_color = 255
        else:
            wi_color = int(np.round(255*((wi[4]-minprob)/(maxprob-minprob))))
        if i_maxprob == ii:
            cv2.rectangle(image,(wi[0],wi[1]), (wi[2],wi[3]), (33, 145, 237), 7)
        cv2.rectangle(image,(wi[0],wi[1]), (wi[2],wi[3]), (0, 0, wi_color), 3)

    if maxp2d!=None: # paint cross
        x, y = maxp2d
        cv2.rectangle(image,(x-5,y-1), (x+5, y+1), (255,255,255), 2)
        cv2.rectangle(image,(x-1,y-5), (x+1, y+5), (255,255,255), 2)
        cv2.rectangle(image,(x-1,y-1), (x+1, y+1), (0,0,0), 1)



def save_data_for_log(collar_scores=None, bbox_ima=None, finddd_scores=None, log_path='experimentos/cloths/', incr_num=True, fignum=1, doshow=False):
    home_path = getenv("HOME")
    if log_path[0]!='/':
        basestr_dir=home_path + log_path
    else:
        basestr_dir= log_path
    system('mkdir -p '+basestr_dir) #create if nonexist
    basestr=basestr_dir+"exp-SIFT_"

    num = get_last_num_log(basestr)
    if incr_num: num+=1

    if collar_scores != None:
        pyplot.ion()
        pyplot.figure(fignum, figsize=(7, 5.5))
        pyplot.clf()
        thismanager = pyplot.get_current_fig_manager()
        thismanager.window.wm_geometry("+1680+610")
        pyplot.imshow(collar_scores)
        pyplot.subplots_adjust(left=0, right=0.9, top=1, bottom=0)
        pyplot.colorbar()
        pyplot.draw()
        pyplot.draw()

        pyplot.savefig(basestr+"%04d_collar_scores.png"%num)
        if doshow:
            pyplot.show()

    if bbox_ima!=None:     
	cv2.imwrite(basestr+"%04d.png"%(num), bbox_ima)
        if doshow:
            #cv2.namedWindow("Grasp point")
            #cv2.imshow("Grasp point", bbox_ima)
            #cv2.waitKey(100)
            pyplot.figure(1099, figsize=(7, 5.5))
            pyplot.clf()
            thismanager = pyplot.get_current_fig_manager()
            thismanager.window.wm_geometry("+2800+610")
            pyplot.imshow(cv2.cvtColor(bbox_ima,cv2.COLOR_BGR2RGB))
            pyplot.subplots_adjust(left=0, right=1, top=1, bottom=0)
            pyplot.draw()
            pyplot.draw()
        if doshow:
            pyplot.show()
    if finddd_scores!=None:
	pyplot.ion()
        cmap=pyplot.cm.jet #spectral
	cmap.set_bad('black',1.)
	masked_array = np.ma.masked_where(np.isnan(finddd_scores), finddd_scores)
        pyplot.figure(fignum, figsize=(7, 5.5))
        pyplot.clf()
        thismanager = pyplot.get_current_fig_manager()
        thismanager.window.wm_geometry("+2240+610")
	pyplot.imshow(masked_array, cmap=cmap)
        pyplot.subplots_adjust(left=0, right=0.9, top=1, bottom=0)
	pyplot.colorbar()
	pyplot.draw()
        pyplot.draw()

	pyplot.savefig(basestr+"%04d_finddd_scores.png"%num)
        if doshow:
            pyplot.show()

        
