#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_wam_controller')
import rospy
from geometry_msgs.msg import PoseStamped
import smach
from position_creator import PositionCreator
from wait_for_msg import WaitForMsgState
from leap_msgs.msg import Leap as Leap_msg

ROBOT_NAME=str(rospy.get_param('/leap_wam/ROBOT_NAME'))

def initialize_cb(message, userdata):
    '''Subscribes to /'+ROBOT_NAME+'/'+ROBOT_NAME+'_controller/libbarrett_link_tcp and outputs the positions to PositionCreator'''
    rospy.loginfo('[LEAP_WAM] Initialize')
    userdata.o_initial_pose_st = message

    return 'succeeded'


def grasp_check_cb(msg, userdata):
    # return False
    if msg.hands_count is 1 and len(msg.hands[0].finger_ids) > 1:
        # return True
        if len(msg.gestures) < 3 and len(msg.gestures) >= 1:
           return 'succeeded'
        else:
            return 'aborted'
    else:
        return 'aborted'


# TODO: CheckForCollision should be implemented by using moveit. 
class CheckForCollision(smach.State):
    '''Publishes the pose for DMP goal'''
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], input_keys=['pose_st_checker','o_pose_st_checker'], output_keys=['o_pose_st_checker'])
        self.pose_z_limit = -0.25

    def execute(self, userdata):
        userdata.o_pose_st_checker = userdata.pose_st_checker
        if userdata.pose_st_checker.pose.position.z < self.pose_z_limit:
            rospy.logwarn('Desired Position is in collision with the table.')
            userdata.o_pose_st_checker.pose.position.z = self.pose_z_limit            
        return 'success'

class PosePublisher(smach.State):
    '''Publishes the pose for DMP goal'''
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], input_keys=['pose_st'])
        self.pose_publish = rospy.Publisher('/pose_st',PoseStamped)

        # Set slower rate for DMP Tracker
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        self.pose_publish.publish(userdata.pose_st)
        self.rate.sleep()
        return 'success'


class GoToDesiredPosition(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['success'],output_keys=['pose_st'])
        self.userdata.moved = False

        with self:

            smach.StateMachine.add('INITIALIZE',WaitForMsgState(
                                                topic='/'+ROBOT_NAME+'/'+ROBOT_NAME+'_controller/libbarrett_link_tcp',
                                                msg_type=PoseStamped,
                                                msg_cb=initialize_cb,
                                                # outcomes=['success','aborted','preempted'],
                                                output_keys=['o_initial_pose_st']),
                transitions={'succeeded':'POSITION_CREATOR','aborted':'INITIALIZE','preempted':'INITIALIZE'},
                remapping={'o_initial_pose_st':'initial'})

            smach.StateMachine.add('POSITION_CREATOR', PositionCreator(),
                    transitions={'success':'CHECK_FOR_COLLISION'},
                    remapping={'end_effector_position':'pose_st_checker'})

            smach.StateMachine.add('CHECK_FOR_COLLISION', CheckForCollision(),
                    transitions={'success':'POSE_PUBLISHER'},
                    remapping={'o_pose_st_checker':'pose_st'})

            smach.StateMachine.add('POSE_PUBLISHER', PosePublisher(),
                    transitions={'success':'GRASP_CHECKER'})

            smach.StateMachine.add('GRASP_CHECKER', WaitForMsgState('/leap/data', Leap_msg, msg_cb=grasp_check_cb),
                  transitions={'succeeded':'success','aborted':'POSITION_CREATOR','preempted':'POSITION_CREATOR'})
