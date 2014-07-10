#!/usr/bin/env python

import roslib; roslib.load_manifest('leap_wam_controller')

import rospy
import threading

import smach


class WaitForMsgState(smach.State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.

It is meant to be extended with a case specific class that initializes this one appropriately
and contains the msg_cb (or overrides execute if really needed).

Its waitForMsg method implements the core functionality: waiting for the message, returning
the message itself or None on timeout.

Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned
message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will
be called with the message and the userdata, so that a self defined method can convert message data to
smach userdata.
Those userdata fields have to be passed via 'output_keys'.

If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
If msg_cb returns a value it should be one of the outcomes. If msg_cb is not existing, execute will behave
as explained above.

If thats still not enough, execute() might be overridden.

latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
timeout: Seconds to wait for a message, defaults to None, disabling timeout
output_keys: Userdata keys that the message callback needs to write to.

Obtained from: https://github.com/felix-kolbe/uashh-rvl-ros-pkg/blob/29f03b13b490c9a477f41e047387443bdbea6e16/uashh_smach/src/util.py#L64
"""

    def __init__(self, topic, msg_type, msg_cb=None, output_keys=None, latch=False, timeout=None, u_execute=None, outcomes=['succeeded','aborted','preempted']):
        if output_keys is None:
            output_keys = []
        smach.State.__init__(self, outcomes=outcomes, output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        """Await and return the message or None on timeout."""
        rospy.loginfo('Waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.loginfo('Got message.')
                message = self.msg

                if not self.latch:
                    self.msg = None

                self.mutex.release()
                return message
            self.mutex.release()

            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('waitForMsg is preempted!')
                return 'preempted'

            rospy.sleep(.1) # TODO: maybe convert ROSInterruptException into valid outcome

        rospy.loginfo('Timeout on waiting for message!')
        return None

    def execute(self, ud):
        """Default simplest execute(), see class description."""
        msg = self.waitForMsg()
        if msg is not None:
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    return cb_result
            return 'succeeded'
        else:
            return 'aborted'
