#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_wam_controller')
import rospy
from leap_msgs.msg import Leap as Leap_msg
from geometry_msgs.msg import PoseStamped, Pose
import smach
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from topic_reader import TopicReaderState
import copy
from scipy import signal

hand_position_list_x = []
hand_position_list_y = []
hand_position_list_z = []
time_loop_start = None
DATA_COLLECTION_TIME = 2
MULTIPLIER = 1.0

def leap_callback(userdata, message):
    ''' leap_callback is a topic_reader callback. It recieves the hand position for (DATA_COLLECTION_TIME) seconds, 
    takes the average, scales and sends to where necessary.
    If a hand is present, it sets (userdata.first_run to True) in order to activate reinitialization of initial pose from
    StayStill -> PositionDifference '''
    
    palm_position_st = PoseStamped()
    palm_pose = Pose()

    global hand_position_list_x
    global hand_position_list_y
    global hand_position_list_z
    global time_loop_start

    data_collection_duration = rospy.Duration.from_sec(DATA_COLLECTION_TIME)
    if message.hands:
        palm_pose = message.hands[0].pose
        rospy.loginfo("[LEAP_WAM] Hands detected....")
        
        hand_position_list_x.append(palm_pose.position.x)
        hand_position_list_y.append(palm_pose.position.y)
        hand_position_list_z.append(palm_pose.position.z)

        # Check if enough time has passed
        if rospy.get_rostime() - time_loop_start < data_collection_duration:
            return 'collect_data'

        rospy.loginfo('[LEAP_WAM] Data collected for '+str(DATA_COLLECTION_TIME)+' second(s)')

        avg_hand_pose_x = sum(hand_position_list_x) / float(len(hand_position_list_x))
        avg_hand_pose_y = sum(hand_position_list_y) / float(len(hand_position_list_y))
        avg_hand_pose_z = sum(hand_position_list_z) / float(len(hand_position_list_z))


        palm_position_st.header.frame_id = 'leap'
        palm_position_st.header.stamp = rospy.Time.now()
        palm_position_st.pose.position.x = (avg_hand_pose_x * MULTIPLIER)
        palm_position_st.pose.position.y = (avg_hand_pose_y * MULTIPLIER)
        palm_position_st.pose.position.z = (avg_hand_pose_z * MULTIPLIER)

        palm_position_st.pose.orientation = palm_pose.orientation
        rospy.loginfo("[LEAP_WAM] Averaged pose from Leap: \n"+str(palm_position_st))
        userdata.o_palm_position = palm_position_st
        userdata.first_run = True
        hand_position_list_x = []
        hand_position_list_y = []
        hand_position_list_z = []
        time_loop_start = rospy.get_rostime()
        return 'success'
    else:
        time_loop_start = rospy.get_rostime()
        rospy.logwarn("[LEAP_WAM] No hands detected!!!")
        return 'empty'


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

        print userdata.first_run
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
    '''Creates the new goal for the robot end effector.

    It finds the difference from the initial hand position and current hand position (averaged) and sends the difference
    to the parent state machine.
    '''
    def __init__(self):
        smach.State.__init__(self,
                             outcomes    = ['success'],
                             input_keys  = ['i_current_hand_pos','i_initial','i_moved_initial','i_stay_put','moved'],
                             output_keys = ['o_end_effector_position','moved','initial'])
        self.end_pose = PoseStamped()
        self.first_run = True
        self.hand_introduced = False
        self.position_list_x = []
        self.position_list_y = []
        self.position_list_z = []

    def execute(self, userdata):
        ''' If hand is removed and reintroduced initial position reinitializes. Otherwise it uses the difference of current 
        hand position and initial hand position to move WAM robot. It only uses the difference not the absolute position. '''

        if userdata.i_stay_put is 1:
            print '\033[93m'+'++++++++++++++++++'+'\033[0m'
            print 'Reinitialize'
            self.hand_introduced = True          
            self.initial_pose = copy.deepcopy(userdata.i_moved_initial)
            return 'success'

        if self.first_run:
            print 'Initialize'
            # previous_hand_pose -> Pose
            self.previous_hand_pose = userdata.i_current_hand_pos.pose
            self.first_run = False
            # initial_pose -> Pose
            self.initial_pose = userdata.i_initial.pose
            userdata.moved = True
            print "FIRST RUN!!!!!!!!!!"
            return 'success'

        elif not userdata.i_stay_put:
            if self.hand_introduced:
                self.previous_hand_pose = userdata.i_current_hand_pos.pose
                self.hand_introduced = False

            print '-------------- GETTING POSITION DIFFERENCE ---------------'
            print ''

            # Set current hand position
            self.current_hand_pose = userdata.i_current_hand_pos.pose
            rospy.loginfo('[LEAP_WAM] Initial position:\n'+str(self.initial_pose))

            # Get the hand position difference and set end effector positon 
            self.end_pose.pose.position.x = self.current_hand_pose.position.x - self.previous_hand_pose.position.x + self.initial_pose.position.x
            self.end_pose.pose.position.y = self.current_hand_pose.position.y - self.previous_hand_pose.position.y + self.initial_pose.position.y
            self.end_pose.pose.position.z = self.current_hand_pose.position.z - self.previous_hand_pose.position.z + self.initial_pose.position.z

            # Get the angle differences. 0.9 is a scale factor to make it more stable
            self.previous_hand_angles = euler_from_quaternion([self.previous_hand_pose.orientation.x, self.previous_hand_pose.orientation.y,
                self.previous_hand_pose.orientation.z, self.previous_hand_pose.orientation.w], 'szyx')
            self.current_hand_angles = euler_from_quaternion([self.current_hand_pose.orientation.x, self.current_hand_pose.orientation.y,
                self.current_hand_pose.orientation.z, self.current_hand_pose.orientation.w], 'szyx')

            self.hand_angle_difference = [0.9 * (x - y) for x, y in zip(self.current_hand_angles, self.previous_hand_angles)]

            # NOTE: Force angle differences to 0 for trying positions.
            # self.hand_angle_difference[0] = 0  # ROLL  --- OK ---
            # self.hand_angle_difference[1] = 0  # PITCH --- OK ---
            # self.hand_angle_difference[2] = 0  # YAW   --- OK ---

            # Set the goal orientation for end-effector
            self.previous_angles = euler_from_quaternion([self.initial_pose.orientation.x,self.initial_pose.orientation.y,
                self.initial_pose.orientation.z, self.initial_pose.orientation.w], 'szyx')

            self.current_angles = [x + y for x, y in zip(self.previous_angles, self.hand_angle_difference)]

            self.current_quat = quaternion_from_euler(self.current_angles[0], self.current_angles[1], self.current_angles[2], 'szyx')

            # Set the new end effector orientation
            self.end_pose.pose.orientation.x = self.current_quat[0]
            self.end_pose.pose.orientation.y = self.current_quat[1]
            self.end_pose.pose.orientation.z = self.current_quat[2]
            self.end_pose.pose.orientation.w = self.current_quat[3]

            self.end_pose.header.frame_id = 'iri_wam_link_base'
            self.end_pose.header.stamp = rospy.Time.now()

            
            # o_end_effector -> PoseStamped end_pose -> PoseStamped
            userdata.o_end_effector_position = self.end_pose

            print '******************** DIFFERENCE ************************'
            print self.current_hand_pose.position.x - self.previous_hand_pose.position.x
            print self.current_hand_pose.position.y - self.previous_hand_pose.position.y
            print self.current_hand_pose.position.z - self.previous_hand_pose.position.z
            print '****************** END DIFFERENCE **********************'
            print self.end_pose
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

            smach.StateMachine.add('READING_LEAP', TopicReaderState(
                                                topic_name='/leap/data',
                                                msg_type=Leap_msg,
                                                callback=leap_callback,
                                                outcomes=['success','empty','aborted','preempted','collect_data'],
                                                output_keys=['o_palm_position','message','first_run']
                                                ), 
                transitions={'success':'POSITION_DIFFERENCE','empty':'CHECK_IF_MOVED','aborted':'READING_LEAP',
                    'preempted':'READING_LEAP','collect_data':'READING_LEAP'},
                remapping={'o_palm_position':'i_current_hand_pos'})

            smach.StateMachine.add('CHECK_IF_MOVED', CheckIfMoved(), 
                transitions={'first_run':'success','stay_still':'STAY_STILL'},
                remapping={'initial_pose':'end_effector_position'})

            smach.StateMachine.add('STAY_STILL', StayStill(), 
                transitions={'reinitialize':'POSITION_DIFFERENCE','success':'success'},
                remapping={'o_stayed_end_effector_position':'end_effector_position','ss_stay_put':'i_stay_put',
                    'o_moved_initial':'i_moved_initial'})

            smach.StateMachine.add('POSITION_DIFFERENCE', PositionDifference(), 
                transitions={'success':'success'},
                remapping={'o_end_effector_position':'end_effector_position', 'moved':'moved','i_initial':'initial'})
