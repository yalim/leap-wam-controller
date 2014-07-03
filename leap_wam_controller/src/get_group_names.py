#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_wam_controller')
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def move_group_python_interface_tutorial():
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    print "============ Robot Groups:"
    print robot.get_group_names()

if __name__ == '__main__':
    print 'tutorial'
    move_group_python_interface_tutorial()