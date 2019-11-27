#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from kobuki_msgs.msg import Led

'''
---
header: 
  seq: 144
  stamp: 
    secs: 1572542583
    nsecs: 526648752
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: -0.426055276895
      y: 0.125823224715
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.672312231795
      w: 0.740267696836
  covariance: [0.005030029466761704, -0.000397074178550387, 0.0, 0.0, 0.0, 0.0, -0.0003970741785503801, 0.002534285750594268, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0017127541766738515]
---

'''


class Localize(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box1', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -0.820429462767
            y: -0.0799067939754
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.714502480706
            w: 0.699632907363
        '''

        self.initial = PoseWithCovarianceStamped()
        self.initial.header.frame_id = "map"
        self.initial.header.stamp = rospy.Time.now()
        self.initial.pose.pose.position.x = -1.40837174498
        self.initial.pose.pose.position.y = -1.08249490241
        self.initial.pose.pose.orientation.x = 0.0
        self.initial.pose.pose.orientation.y = 0.0
        self.initial.pose.pose.orientation.z = -0.714502480706
        self.initial.pose.pose.orientation.w = 0.699632907363

        self.callbacks = callbacks
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        #self.initial_pub.publish(self.initial)

        start = time.time()

        while time.time() - start < 5:
            self.twist.linear.x = 0.5
            self.twist.angular.z = -0.3
            self.cmd_vel_pub.publish(self.twist)

            if shutdown_requested:
                return 'done4'

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        target_heading = (self.callbacks.heading + 350) % 360

        turning = True
        previous_difference = None
        while turning:
            if shutdown_requested:
                return 'done4'
            difference = minimum_angle_between_headings(target_heading, self.callbacks.heading)

            if previous_difference is None:
                self.twist.angular.z = 0.4
                self.cmd_vel_pub.publish(self.twist)
            else:
                if difference < 1:
                    turning = False
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    self.twist.angular.z = 0.4
                    self.cmd_vel_pub.publish(self.twist)

            if previous_difference != difference:
                previous_difference = difference

        return 'box1'


class Box1(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -1.52921782505
            y: -0.964718068027
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: 0.824532695146
            w: 0.56581431109

        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.54587139462
        self.target.target_pose.pose.position.y = -0.972597373004
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = 0.824532695146
        self.target.target_pose.pose.orientation.w = 0.56581431109

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 1:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box2'


class Box2(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box3', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -2.36646936837
            y: -1.27823477483
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.991971583453
            w: 0.126460972722
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -2.36646936837
        self.target.target_pose.pose.position.y = -1.27823477483
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.991971583453
        self.target.target_pose.pose.orientation.w = 0.126460972722

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 2:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box3'


class Box3(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box4', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server

        '''
        position: 
            x: -2.08344307577
            y: -2.06366635946
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.988164547126
            w: 0.15339761342

        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -2.08344307577
        self.target.target_pose.pose.position.y = -2.06366635946
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.988164547126
        self.target.target_pose.pose.orientation.w = 0.15339761342

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 3:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box4'


class Box4(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box5', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -0.82371839872
            y: -2.01898909535
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: 0.110374592646
            w: 0.993890058959
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.82371839872
        self.target.target_pose.pose.position.y = -2.01898909535
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = 0.110374592646
        self.target.target_pose.pose.orientation.w = 0.993890058959

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 4:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box5'


class Box5(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box6', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -0.534573222531
            y: -2.64808881431
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: 0.0599241194844
            w: 0.998202935231
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.534573222531
        self.target.target_pose.pose.position.y = -2.64808881431
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = 0.0599241194844
        self.target.target_pose.pose.orientation.w = 0.998202935231

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 5:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box6'


class Box6(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box7', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -1.88168899969
            y: -2.66331720019
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.989069018119
            w: 0.147453305823
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.88168899969
        self.target.target_pose.pose.position.y = -2.66331720019
        #self.target.target_pose.pose.position.x = -0.344961246087
        #self.target.target_pose.pose.position.y = -4.06018585631
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.989069018119
        self.target.target_pose.pose.orientation.w = 0.147453305823

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 6:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box7'


class Box7(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -1.70333515976
            y: -3.35096111794
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.978520415773
            w: 0.206149935519
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.67067776913
        self.target.target_pose.pose.position.y = -3.4407324293
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.978520415773
        self.target.target_pose.pose.orientation.w = 0.206149935519

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 7:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box8'


class Box8(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'return'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        '''
        position: 
            x: -1.40528647143
            y: -3.94364406666
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.985633442886
            w: 0.168898538375
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.38042785743
        self.target.target_pose.pose.position.y = -4.02047473645
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.985633442886
        self.target.target_pose.pose.orientation.w = 0.168898538375

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub.publish(1)  # green
            time.sleep(5)
            self.led_pub.publish(0)  # off
        elif self.callbacks.target_box == 8:
            self.led_pub.publish(3)  # red
            time.sleep(5)
            self.led_pub.publish(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'return'


class Return(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['success4', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        '''
        position: 
            x: 0.857642769587
            y: -3.15814141971
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: 0.168549594567
            w: 0.985693174457
        '''

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = 0.857642769587
        self.target.target_pose.pose.position.y = -3.15814141971
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = 0.168549594567
        self.target.target_pose.pose.orientation.w = 0.985693174457

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        return 'success4'

def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
    return heading_difference


def get_state_machine(callbacks):

    # Create done outcome which will stop the state machine
    sm_event_4 = smach.StateMachine(outcomes=['DONE4', 'SUCCESS4'])

    with sm_event_4:
        smach.StateMachine.add('LOCALIZE', Localize(callbacks),
                               transitions={'box1': 'BOX1', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX1', Box1(callbacks),
                               transitions={'box2': 'BOX2', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX2', Box2(callbacks),
                               transitions={'box3': 'BOX3', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX3', Box3(callbacks),
                               transitions={'box4': 'BOX4', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX4', Box4(callbacks),
                               transitions={'box5': 'BOX5', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX5', Box5(callbacks),
                               transitions={'box6': 'BOX6', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX6', Box6(callbacks),
                               transitions={'box7': 'BOX7', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX7', Box7(callbacks),
                               transitions={'box8': 'BOX8', 'done4': 'DONE4'})
        smach.StateMachine.add('BOX8', Box8(callbacks),
                               transitions={'done4': 'DONE4', 'return': 'RETURN'})
        smach.StateMachine.add('RETURN', Return(callbacks),
                               transitions={'success4': 'SUCCESS4', 'done4': 'DONE4'})
    return sm_event_4


'''
header: 
  seq: 125
  stamp: 
    secs: 1571336109
    nsecs: 238852016
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: -1.4029582974
      y: -0.161230396952
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.409603401919
      w: 0.912263697149
  covariance: [0.003383795001103751, 0.0030187297400489366, 0.0, 0.0, 0.0, 0.0, 0.0030187297400489366, 0.0106325757067692, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0017897886634780384]
'''