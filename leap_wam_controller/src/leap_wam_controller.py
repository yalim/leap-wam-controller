#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_wam_controller')
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import smach
import smach_ros
from go_to_desired_pose import GoToDesiredPosition
from go_grasping_point import DetectCollarAndGo
from iri_common_drivers_msgs.msg import tool_openAction, tool_closeAction
import os
import csv
import rospkg
import math

rospack = rospkg.RosPack()

PACKAGE_PATH = rospack.get_path('leap_wam_controller')
ROBOT_NAME=str(rospy.get_param('/leap_wam/ROBOT_NAME'))
CSV_PATH = PACKAGE_PATH + '/config/grasping_position_list.csv'

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
        # self.quat = quaternion_from_euler(self.x, self.y, self.z)
        # self.initial_pose.orientation.x = self.quat[0]
        # self.initial_pose.orientation.y = self.quat[1]
        # self.initial_pose.orientation.z = self.quat[2]
        # self.initial_pose.orientation.w = self.quat[3]

        self.initial_pose.orientation.x = 0.00107
        self.initial_pose.orientation.y = 0.84099
        self.initial_pose.orientation.z = 0.00053
        self.initial_pose.orientation.w = 0.54104

        self.initial_pose_st.pose = self.initial_pose
        self.initial_pose_st.header.frame_id = ROBOT_NAME+'_link_base'
        self.initial_pose_st.header.stamp = rospy.Time.now()

        userdata.o_initial_pose_st = self.initial_pose_st

        return 'success'

class RecordPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success'],input_keys=['pose_st'])
        self.position_list = []

    def execute(self, userdata):
        rospy.loginfo('Recording position .....')
        self.position_list = [userdata.pose_st.pose.position.x,
                         userdata.pose_st.pose.position.y,
                         userdata.pose_st.pose.position.z,
                         - math.asin(2 * userdata.pose_st.pose.orientation.x * userdata.pose_st.pose.orientation.y)]

        if os.path.exists(CSV_PATH):
            fd = open(CSV_PATH,'a')
            self.writer2 = csv.writer(fd)
            self.writer2.writerow(self.position_list)
            fd.close()
        else:
            f = open(CSV_PATH,'w')
            self.writer = csv.writer(f)
            self.writer.writerow(self.position_list)
            f.close()

        rospy.loginfo('Position saved')
        return 'success'

class Lift(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success', 'aborted'])
        self.lift_pose_publisher = rospy.Publisher('/pose_st', PoseStamped)
        self.lift_pose = PoseStamped()


    def execute(self, userdata):
        self.lift_pose.header.frame_id = ROBOT_NAME+'_link_base'
        self.lift_pose.header.stamp = rospy.Time.now()
        self.lift_pose.pose.position.x = 0.4
        self.lift_pose.pose.position.y = 0.0
        self.lift_pose.pose.position.z = 0.44

        self.lift_pose.pose.orientation.x = 0.00107
        self.lift_pose.pose.orientation.y = 0.84099
        self.lift_pose.pose.orientation.z = 0.00053
        self.lift_pose.pose.orientation.w = 0.54104

        self.lift_pose_publisher.publish(self.lift_pose)
        rospy.loginfo('Lifted')
        return 'success'


def main():
    rospy.init_node('leap_wam_controller')
    sm = smach.StateMachine(outcomes=['aborted','success'])
    with sm:
        sm.userdata.moved = False

        smach.StateMachine.add('INITIALIZE', DetectCollarAndGo(),
            transitions={'finishOk':'OPEN_GRIPPER','failedVision':'INITIALIZE'})

        smach.StateMachine.add('OPEN_GRIPPER', smach_ros.SimpleActionState('/'+ROBOT_NAME+'/gripper/tool_open_action',tool_openAction),
            transitions={'succeeded':'GO_TO_DESIRED_POSE','preempted':'OPEN_GRIPPER','aborted':'OPEN_GRIPPER'})

        smach.StateMachine.add('GO_TO_DESIRED_POSE', GoToDesiredPosition(),
            transitions={'success':'RECORD_POSITION'})

        smach.StateMachine.add('RECORD_POSITION', RecordPosition(),
            transitions={'success':'GRASP'})

        smach.StateMachine.add('GRASP', smach_ros.SimpleActionState('/'+ROBOT_NAME+'/gripper/tool_close_action',tool_closeAction),
            transitions={'succeeded':'LIFT','preempted':'GRASP','aborted':'GRASP'})

        smach.StateMachine.add('LIFT', Lift(),
            transitions={'success':'success','aborted':'LIFT'})

    sis = smach_ros.IntrospectionServer('wam_leap_control', sm, '/SM_WAM_LEAP')
    sis.start()

    outcome = sm.execute()

if __name__ == '__main__':
    main()
