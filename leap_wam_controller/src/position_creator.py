#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_wam_controller')
import rospy
from leap_msgs.msg import Leap as Leap_msg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32MultiArray
import smach
from wait_for_msg import WaitForMsgState
from visualization_msgs.msg import Marker
import copy
from scipy import signal
import numpy as np
import math

hands_stable = False
ROBOT_NAME=str(rospy.get_param('/leap_wam/ROBOT_NAME'))


def leap_callback(message, userdata):
    ''' This is a topic_reader callback. It recieves the hand position, scales and sends to where necessary.

    If a hand is present, it sets (userdata.first_run to True) in order to activate reinitialization of initial pose from
    StayStill -> PositionDifference '''
    global hands_stable
    palm_position_st = PoseStamped()
    palm_pose = Pose()
    if message.hands_count is 1 and len(message.hands[0].finger_ids) > 2:# and message.hands[0].sphere_radius > 60:
        palm_pose = message.hands[0].pose
        rospy.loginfo("[LEAP_WAM] Hands detected....")
        rospy.loginfo("[LEAP_WAM] Recieved pose from Leap: \n"+str(message.hands[0].pose))
        palm_position_st.header.frame_id = ''
        palm_position_st.header.stamp = rospy.Time.now()
        palm_position_st.pose.position = palm_pose.position
        palm_position_st.pose.orientation = palm_pose.orientation
        userdata.o_palm_position = palm_position_st
        userdata.direction = message.hands[0].direction_yaw
        userdata.first_run = True
        return 'success'
    else:
        rospy.logwarn("[LEAP_WAM] No hands or more than one hand !!!")
        hands_stable = False
        return 'empty'

class StandardDeviation(smach.State):
    '''Makes robot stay still until the hand is stable.

    This takes the standard deviation of hand position and waits until a threshold is met. The default value is 5.
    It also publishes a marker in rviz to indicate if the hand is stable. (RED: Not stable. GREEN: Stable'''
    def __init__(self, threshold=5):
        smach.State.__init__(self,
            outcomes = ['success','hand_not_stable'],
            input_keys = ['i_palm_position'],
            output_keys = ['o_current_hand_pos'])
        self.pose_list_x = []
        self.pose_list_y = []
        self.pose_list_z = []
        self.std_publish = rospy.Publisher('/leap_wam_controller/std_dev',Float32MultiArray)
        self.marker_publish = rospy.Publisher('visualization_marker', Marker)
        self.marker = Marker()
        self._threshold = threshold
        self.leap_marker = Marker()
        self.leap_marker_publisher = rospy.Publisher('visualization_marker_leap', Marker)

    def execute(self, userdata):
        global hands_stable
        # Create a marker in rviz to indicate it is ready to move hand
        self.marker.header.frame_id = ROBOT_NAME+'_link_base'
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = 'hand_leap'
        self.marker.id = 0
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose = userdata.i_palm_position.pose
        self.marker.pose.position.x = self.marker.pose.position.x + 1.0
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.02
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.leap_marker.header.frame_id = ROBOT_NAME+'_link_base'
        self.leap_marker.header.stamp = rospy.Time.now()
        self.leap_marker.ns = 'leap'
        self.leap_marker.id = 1
        self.leap_marker.type = Marker.CUBE
        self.leap_marker.action = Marker.ADD
        self.leap_marker.pose.position.x = 0
        self.leap_marker.pose.position.y = 0
        self.leap_marker.pose.position.z = 0
        self.leap_marker.pose.orientation.x = 0
        self.leap_marker.pose.orientation.y = 0
        self.leap_marker.pose.orientation.z = 0
        self.leap_marker.pose.orientation.w = 0
        self.leap_marker.scale.x = 0.02
        self.leap_marker.scale.y = 0.1
        self.leap_marker.scale.z = 0.01
        self.leap_marker.color.r = 0.0
        self.leap_marker.color.g = 0.0
        self.leap_marker.color.b = 0.0
        self.leap_marker.color.a = 1.0
        self.leap_marker_publisher.publish(self.leap_marker)

        self.pose_list_x.append(userdata.i_palm_position.pose.position.x*1000)
        self.pose_list_y.append(userdata.i_palm_position.pose.position.y*1000)
        self.pose_list_z.append(userdata.i_palm_position.pose.position.z*1000)
        
        if len(self.pose_list_x)>10:
            self.pose_list_x = self.pose_list_x[-10:]
            self.pose_list_y = self.pose_list_x[-10:]
            self.pose_list_z = self.pose_list_x[-10:]

        self.pose_arr_x = np.array(self.pose_list_x)
        self.pose_arr_y = np.array(self.pose_list_y)
        self.pose_arr_z = np.array(self.pose_list_z)
        print self.pose_list_x

        self.std_x = np.std(self.pose_arr_x)
        self.std_y = np.std(self.pose_arr_y)
        self.std_z = np.std(self.pose_arr_z)
        self.std = [self.std_x, self.std_y, self.std_z]

        self.msg = Float32MultiArray()
        self.msg.data = self.std
        if not hands_stable:
            if len(self.pose_list_x)<2 or max(self.std) > self._threshold:
                rospy.logwarn('[LEAP_WAM] Hand is not stable')
                self.marker.color.r = 1.0
                self.marker.color.g = 0.0
                self.marker_publish.publish(self.marker)
                return 'hand_not_stable'
            elif max(self.std) < self._threshold:
                self.marker_publish.publish(self.marker)

        self.marker.action = Marker.ADD
        self.marker_publish.publish(self.marker)
        self.std_publish.publish(self.msg)
        userdata.o_current_hand_pos = userdata.i_palm_position
        hands_stable = True
        return 'success'


class StayStill(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['success','reinitialize'],
            input_keys = ['end_effector_position','first_run'],
            output_keys = ['o_stayed_end_effector_position','o_moved_initial','ss_stay_put','first_run'])

        self.first_run = True
    def execute(self, userdata):
        ''' If the robot moved, StayStill firstly goes to PositionDifference in order to set new end effector position as
        initial position. After the first run it sends current end effector position to parent to make it stay still. '''

        # print userdata.first_run
        if userdata.first_run:
            userdata.ss_stay_put = 1
            self.moved_initial_var = userdata.end_effector_position
            userdata.o_moved_initial = copy.deepcopy(self.moved_initial_var.pose)
            userdata.o_stayed_end_effector_position = userdata.end_effector_position
            userdata.first_run = False
            return 'reinitialize'            
        else:
            userdata.ss_stay_put = 0
            return 'success'


class PositionDifference(smach.State):
    def __init__(self, scale=0.5):
        smach.State.__init__(self,
                             outcomes    = ['success'],
                             input_keys  = ['i_current_hand_pos','i_initial','i_moved_initial','i_stay_put','moved','direction'],
                             output_keys = ['o_end_effector_position','moved','initial']) #removed moved,initial
        self.end_pose = PoseStamped()
        self.first_run = True
        self.hand_introduced = False
        self.position_list_x = []
        self.position_list_y = []
        self.position_list_z = []
        self.orientation_list_x = []
        self.orientation_list_y = []
        self.orientation_list_z = []
        self.orientation_list_w = []

        self.scaling = scale
        self.initial_angle = 0

    def execute(self, userdata):
        ''' If hand is removed and reintroduced initial position reinitializes. Otherwise it uses the difference of current 
        hand position and initial hand position to move WAM robot. It only uses the difference not the absolute position. '''

        if userdata.i_stay_put is 1:
            print '\033[93m'+'++++++++++++++++++'+'\033[0m'
            print 'Reinitialize'
            self.hand_introduced = True          
            self.initial_pose = copy.deepcopy(userdata.i_moved_initial)
            self.initial_angle = - math.asin(2 * userdata.i_moved_initial.orientation.x * userdata.i_moved_initial.orientation.y)
            print self.initial_angle
            return 'success'

        if self.first_run:
            # Initialize the arrays for filtering
            self.position_list_x = 10*[userdata.i_initial.pose.position.x]
            self.position_list_y = 10*[userdata.i_initial.pose.position.y]
            self.position_list_z = 10*[userdata.i_initial.pose.position.z]
            self.orientation_list_x = 10*[userdata.i_initial.pose.orientation.x]
            self.orientation_list_y = 10*[userdata.i_initial.pose.orientation.y]
            self.orientation_list_z = 10*[userdata.i_initial.pose.orientation.z]
            self.orientation_list_w = 10*[userdata.i_initial.pose.orientation.w]
            print 'Initialized'
            # previous_hand_pose -> Pose
            self.previous_hand_pose = userdata.i_current_hand_pos.pose
            self.first_run = False
            # initial_pose -> Pose
            self.initial_pose = userdata.i_initial.pose
            userdata.o_end_effector_position = userdata.i_initial
            self.initial_direction = userdata.direction
            userdata.moved = True
            return 'success'

        elif not userdata.i_stay_put:
            if self.hand_introduced:
                self.previous_hand_pose = userdata.i_current_hand_pos.pose
                self.initial_direction = userdata.direction
                print 'INITIAL_DIRECTION_RESET'
                print self.initial_direction
                self.hand_introduced = False

            print '-------------- GETTING POSITION DIFFERENCE ---------------'
            print ''

            # Set current hand position
            self.current_direction = userdata.direction
            self.current_hand_pose = userdata.i_current_hand_pos.pose
            # rospy.loginfo('[LEAP_WAM] Previous hand position:\n'+str(self.previous_hand_pose))
            # rospy.loginfo('[LEAP_WAM] Current hand position:\n'+str(self.current_hand_pose))
            rospy.loginfo('[LEAP_WAM] Initial position:\n'+str(self.initial_pose))

            # Get the hand position difference and set end effector positon 
            self.end_pose.pose.position.x = self.scaling * (self.current_hand_pose.position.x - self.previous_hand_pose.position.x) + self.initial_pose.position.x
            self.end_pose.pose.position.y = self.scaling * (self.current_hand_pose.position.y - self.previous_hand_pose.position.y) + self.initial_pose.position.y
            self.end_pose.pose.position.z = self.scaling * (self.current_hand_pose.position.z - self.previous_hand_pose.position.z) + self.initial_pose.position.z

            #Filter position x, y, z
            self.position_list_x.append(self.end_pose.pose.position.x)
            self.position_list_y.append(self.end_pose.pose.position.y)
            self.position_list_z.append(self.end_pose.pose.position.z)

            if len (self.position_list_x) > 10:
                b, a = signal.butter(1, 0.5, 'low')
                output_signal_x = signal.lfilter(b, a, self.position_list_x)
                output_signal_y = signal.lfilter(b, a, self.position_list_y)
                output_signal_z = signal.lfilter(b, a, self.position_list_z)

                self.end_pose.pose.position.x = round(output_signal_x[-1], 3)
                self.end_pose.pose.position.y = round(output_signal_y[-1], 3)
                self.end_pose.pose.position.z = round(output_signal_z[-1], 3)

            self.angle_difference = - round((self.initial_angle + self.current_direction - self.initial_direction),3)
            print 'AAAAA'
            print self.angle_difference

            # Set the new end effector orientation
            self.end_pose.pose.orientation.x = math.sin(self.angle_difference / 2)
            self.end_pose.pose.orientation.y = math.cos(self.angle_difference / 2)
            self.end_pose.pose.orientation.z = 0
            self.end_pose.pose.orientation.w = 0
            # Filter orientation x, y, z, w 
            self.orientation_list_x.append(self.end_pose.pose.orientation.x)
            self.orientation_list_y.append(self.end_pose.pose.orientation.y)
            self.orientation_list_z.append(self.end_pose.pose.orientation.z)
            self.orientation_list_w.append(self.end_pose.pose.orientation.w)

            if len(self.orientation_list_x) > 10:
                b, a = signal.butter(1, 0.2, 'low')

                output_signal_o_x = signal.lfilter(b, a, self.orientation_list_x)
                self.end_pose.pose.orientation.x = round(output_signal_o_x[-1], 3)

                output_signal_o_y = signal.lfilter(b, a, self.orientation_list_y)
                self.end_pose.pose.orientation.y = round(output_signal_o_y[-1], 3)

                output_signal_o_z = signal.lfilter(b, a, self.orientation_list_z)
                self.end_pose.pose.orientation.z = round(output_signal_o_z[-1], 3)

                output_signal_o_w = signal.lfilter(b, a, self.orientation_list_w)
                self.end_pose.pose.orientation.w = round(output_signal_o_w[-1], 3)

            self.end_pose.header.frame_id = ROBOT_NAME+'_link_base'
            self.end_pose.header.stamp = rospy.Time.now()

            
            # o_end_effector -> PoseStamped 
            # end_pose -> PoseStamped
            userdata.o_end_effector_position = self.end_pose

            # print '******************** DIFFERENCE ************************'
            # print self.current_hand_pose.position.x - self.previous_hand_pose.position.x
            # print self.current_hand_pose.position.y - self.previous_hand_pose.position.y
            # print self.current_hand_pose.position.z - self.previous_hand_pose.position.z
            # print '****************** END DIFFERENCE **********************'
            # print self.end_pose
            print '-------------- POSITION DIFFERENCE IS SET ---------------'
            print ''

            return 'success'


class CheckIfMoved(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['first_run','stay_still'], input_keys=['moved','initial'], 
            output_keys=['initial_pose','stay_put'])
        self.is_moved = False

    def execute(self,userdata):
        ''' This checks if the robot moved from the first start. If moved it will go to StayStill, else it returns to
         parent and make the robot stay in its initial positon. '''

        if userdata.moved:
            self.is_moved = True
        if not self.is_moved:
            print 'Not moved'
            userdata.initial_pose = userdata.initial
            return 'first_run'
        else:
            print 'Moved!!!'
            userdata.stay_put = True
            return 'stay_still'


class PositionCreator(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['success'],
                                    input_keys=['initial'],
                                    output_keys=['end_effector_position'])

        self.userdata.moved = False
        self.userdata.i_stay_put = 0

        with self:

            # smach.StateMachine.add('READING_LEAP', TopicReaderState(
            #                                     topic_name='/leap/data',
            #                                     msg_type=Leap_msg,
            #                                     callback=leap_callback,
            #                                     outcomes=['success','empty','aborted','preempted'],
            #                                     output_keys=['o_palm_position','message','first_run']
            #                                     ), 
            #     transitions={'success':'STD_DEV','empty':'CHECK_IF_MOVED','aborted':'READING_LEAP',
            #         'preempted':'READING_LEAP'},
            #     remapping={'o_palm_position':'i_palm_position'})

            smach.StateMachine.add('READING_LEAP', WaitForMsgState(
                                                topic='/leap/data',
                                                msg_type=Leap_msg,
                                                msg_cb=leap_callback,
                                                outcomes=['success','empty','aborted','preempted'],
                                                output_keys=['o_palm_position','first_run','message','direction'] #removed message 
                                                ), 
                transitions={'success':'STD_DEV','empty':'CHECK_IF_MOVED','aborted':'READING_LEAP',
                    'preempted':'READING_LEAP'},
                remapping={'o_palm_position':'i_palm_position'})

            smach.StateMachine.add('STD_DEV', StandardDeviation(threshold=1), 
                transitions={'success':'POSITION_DIFFERENCE','hand_not_stable':'READING_LEAP'},
                remapping={'o_current_hand_pos':'i_current_hand_pos'})

            smach.StateMachine.add('CHECK_IF_MOVED', CheckIfMoved(), 
                transitions={'first_run':'success','stay_still':'STAY_STILL'},
                remapping={'initial_pose':'end_effector_position'})

            smach.StateMachine.add('STAY_STILL', StayStill(), 
                transitions={'reinitialize':'POSITION_DIFFERENCE','success':'success'},
                remapping={'o_stayed_end_effector_position':'end_effector_position','ss_stay_put':'i_stay_put',
                    'o_moved_initial':'i_moved_initial'})

            smach.StateMachine.add('POSITION_DIFFERENCE', PositionDifference(scale=0.5), 
                transitions={'success':'success'},
                remapping={'o_end_effector_position':'end_effector_position', 'moved':'moved','i_initial':'initial'})
