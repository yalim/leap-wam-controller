#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_wam_controller')
import rospy
from iri_wam_smach.st_get_joints_from_pose import GetJointsFromPose
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import smach
import smach_ros
from tf.transformations import quaternion_from_euler
from position_creator import PositionCreator
from topic_reader import TopicReaderState
from scipy import signal
from leap_wam_controller.srv import LeapGoalPose

class InitialPosition(smach.State):
    '''Sends the robot to an initial position.

    Initial position should be set to self.initial_pose which is type geometry_msgs/Pose. The output
    output of this state is geometry_msgs/PoseStamped type with name initial_pose_st '''
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success','aborted'], output_keys = ['o_initial_pose_st'])

        self.initial_pose = Pose()
        self.initial_pose_st = PoseStamped()
        self.x = 0.0
        self.y = 3.14
        self.z = 3.14/2.0 - 0.01

    def execute(self, userdata):
        rospy.loginfo("[LEAP_WAM] Robot to initial position")
        self.initial_pose.position.x = 0.4
        self.initial_pose.position.y = 0.0
        self.initial_pose.position.z = 0.44
        self.quat = quaternion_from_euler(self.x, self.y, self.z)
        self.initial_pose.orientation.x = self.quat[0]
        self.initial_pose.orientation.y = self.quat[1]
        self.initial_pose.orientation.z = self.quat[2]
        self.initial_pose.orientation.w = self.quat[3]

        # self.initial_pose.orientation.x = 0.00107
        # self.initial_pose.orientation.y = 0.84099
        # self.initial_pose.orientation.z = 0.00053
        # self.initial_pose.orientation.w = 0.54104

        self.initial_pose_st.pose = self.initial_pose
        self.initial_pose_st.header.frame_id = 'iri_wam_link_base'
        self.initial_pose_st.header.stamp = rospy.Time.now()

        userdata.o_initial_pose_st = self.initial_pose_st

        return 'success'

def initialize_cb(userdata, message):
    '''Subscribes to /iri_wam/iri_wam_controller/libbarrett_link_tcp and outputs the positions to PositionCreator'''
    rospy.loginfo('[LEAP_WAM] Initialize')
    userdata.o_initial_pose_st = message

    return 'success'


class PosePublisher(smach.State):
    '''Publishes the pose for DMP goal'''
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'], input_keys=['pose_st'])
        self.pose_publish = rospy.Publisher('/leap_wam_controller/pose_st',PoseStamped)

        # Set slower rate for DMP Tracker
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        # rospy.wait_for_service('leap_wam_collision_checker_server')
        # check_for_collision = rospy.ServiceProxy('leap_wam_collision_checker_server', LeapGoalPose)
        # result = check_for_collision(userdata.pose_st.pose)
        # print 'Is there collision? ' + str(result.result)
        self.pose_publish.publish(userdata.pose_st)
        self.rate.sleep()

        return 'success'


class JointPublisher(smach.State):
    ''' Publishes filtered joint trajectory for simulation.

    This part also uses a low pass filter in order to avoid abrupt jumps at joint trajectories.'''

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['success','aborted'],
            input_keys = ['i_joints_to_position'])
        self.pub = rospy.Publisher('/joint_traj',JointTrajectory)
        # self.publishDMP = rospy.Publisher('/leap_wam_controller/DMPTrackerNewGoal', JointTrajectoryPoint)
        self.all_sets_of_joint_coordinates = JointTrajectory()
        self.one_set_of_joint_coordinates = JointTrajectoryPoint()
        self.number_of_joints = 7
        self.time_for_move = 0.001
        # Create  J1 through J7
        for joint_number in range (1, self.number_of_joints + 1):  
            self.all_sets_of_joint_coordinates.joint_names.append("J%i"%joint_number)
        self.traj = []

    def execute(self,userdata):

        # CREATE JOINT TRAJECTORIES
        self.all_sets_of_joint_coordinates.header.stamp = rospy.Time.now()
        self.one_set_of_joint_coordinates.positions = userdata.i_joints_to_position.position
        # self.one_set_of_joint_coordinates.velocities = 7*[0.1]
        self.one_set_of_joint_coordinates.time_from_start = rospy.Duration.from_sec(self.time_for_move)
        self.all_sets_of_joint_coordinates.points.append(self.one_set_of_joint_coordinates)

        # FILTER Joints
        # if len(self.all_sets_of_joint_coordinates.points) > 0:
        #     self.traj.append(self.all_sets_of_joint_coordinates.points[-1].positions)
        #     b, a = signal.butter(2, 0.2, 'low')
        #     j1_in = [j1[0] for j1 in self.traj]
        #     j2_in = [j2[1] for j2 in self.traj]
        #     j3_in = [j3[2] for j3 in self.traj]
        #     j4_in = [j4[3] for j4 in self.traj]
        #     j5_in = [j5[4] for j5 in self.traj]
        #     j6_in = [j6[5] for j6 in self.traj]
        #     j7_in = [j7[6] for j7 in self.traj]

        #     output_signal_J1 = signal.lfilter(b, a, j1_in)
        #     output_signal_J2 = signal.lfilter(b, a, j2_in)
        #     output_signal_J3 = signal.lfilter(b, a, j3_in)
        #     output_signal_J4 = signal.lfilter(b, a, j4_in)
        #     output_signal_J5 = signal.lfilter(b, a, j5_in)
        #     output_signal_J6 = signal.lfilter(b, a, j6_in)
        #     output_signal_J7 = signal.lfilter(b, a, j7_in)

        #     self.all_sets_of_joint_coordinates.points[-1].positions = [output_signal_J1[-1], output_signal_J2[-1], output_signal_J3[-1], 
                                            # output_signal_J4[-1], output_signal_J5[-1], output_signal_J6[-1], output_signal_J7[-1]]

        self.pub.publish(self.all_sets_of_joint_coordinates)
        # self.publishDMP.publish(self.all_sets_of_joint_coordinates.points[-1])
        return 'success'


def main():
    rospy.init_node('leap_wam_controller')
    sm = smach.StateMachine(outcomes=['aborted'])
    with sm:
        sm.userdata.moved = False
        # smach.StateMachine.add('INITIALIZE',InitialPosition(),
        #     transitions={'success':'POSITION_CREATOR','aborted':'INITIALIZE'},
        #     remapping={'o_initial_pose_st':'initial'})

        smach.StateMachine.add('INITIALIZE',TopicReaderState(
                                                topic_name='/iri_wam/iri_wam_controller/libbarrett_link_tcp',
                                                msg_type=PoseStamped,
                                                callback=initialize_cb,
                                                outcomes=['success','aborted','preempted'],
                                                output_keys=['o_initial_pose_st']),
            transitions={'success':'POSITION_CREATOR','aborted':'INITIALIZE','preempted':'INITIALIZE'},
            remapping={'o_initial_pose_st':'initial'})

        smach.StateMachine.add('POSITION_CREATOR', PositionCreator(),
            # transitions={'success':'WAM_IK'},
            transitions={'success':'POSE_PUBLISHER'},
            remapping={'end_effector_position':'pose_st'})

        smach.StateMachine.add('POSE_PUBLISHER', PosePublisher(),
            transitions={'success':'POSITION_CREATOR'}
            # transitions={'success':'WAM_IK'}
            )

        smach.StateMachine.add('WAM_IK', GetJointsFromPose('/iri_wam_ik/wamik'),
            transitions={'success':'JOINT_PUBLISHER','empty':'POSITION_CREATOR','no_kinematic_solution':'POSITION_CREATOR'},
            remapping={'joints_to_position':'i_joints_to_position'})

        smach.StateMachine.add('JOINT_PUBLISHER', JointPublisher(),
            transitions={'success':'POSITION_CREATOR','aborted':'POSITION_CREATOR'})

        # smach.StateMachine.add('WAM_IK', SM_WAM_MoveFromPose('/move_iri_wam', '/iri_wam_ik/wamik'),
        #     transitions={'success':'READING_LEAP','aborted':'READING_LEAP','no_kinematic_solution':'READING_LEAP'})

    sis = smach_ros.IntrospectionServer('wam_leap_control', sm, '/SM_WAM_LEAP')
    sis.start()

    outcome = sm.execute()

if __name__ == '__main__':
    main()
