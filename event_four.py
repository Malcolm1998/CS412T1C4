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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import trig
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound


class Localize(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'find_markers', 'box2', 'box8'])

        self.initial = PoseWithCovarianceStamped()

        self.callbacks = callbacks
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        #self.notified_box_found = False
        #self.notified_box_target_found = False

    def execute(self, userdata):
        global shutdown_requested

        start = time.time()

        while time.time() - start < 5:
            self.twist.linear.x = 0.5
            self.twist.angular.z = -0.3
            self.cmd_vel_pub.publish(self.twist)

            if shutdown_requested:
                return 'done4'

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        target_heading = (self.callbacks.bot_odom_heading + 350) % 360

        self.initial = self.callbacks.bot_map_pose
        self.initial.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        self.initial_pub.publish(self.initial)

        turning = True
        previous_difference = None
        while turning:
            if shutdown_requested:
                return 'done4'

            difference = trig.minimum_angle_between_headings(target_heading, self.callbacks.bot_odom_heading)

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

        if self.callbacks.box_position is not None:
            self.led2_pub.publish(3)  # red
            self.sound_pub.publish(1)
            time.sleep(1)

        if self.callbacks.box_target_position is not None:
            self.led1_pub.publish(1)  # Green
            self.sound_pub.publish(1)
            time.sleep(1)

        if shutdown_requested:
            return 'done4'

        if self.callbacks.box_target_position is None or self.callbacks.box_position is None:
            return 'find_markers'
        else:
            self.led2_pub.publish(3)  # Off
            self.led1_pub.publish(1)  # Off
            distance_from_box_target = trig.get_distance(self.callbacks.bot_map_position, self.callbacks.box_target_position)
            distance_from_box = trig.get_distance(self.callbacks.bot_map_position, self.callbacks.box_position)
            if distance_from_box < distance_from_box_target:
                return 'box2'
            else:
                return 'box8'


class FindMarkers(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'box1', 'box2', 'box8'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

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
        self.led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if shutdown_requested:
            return 'done4'

        if self.callbacks.box_position is not None:
            self.led2_pub.publish(3)  # red
            self.sound_pub.publish(1)
            time.sleep(1)

        if self.callbacks.box_target_position is not None:
            self.led1_pub.publish(1)  # Green
            self.sound_pub.publish(1)
            time.sleep(1)

        if self.callbacks.box_target_position is None or self.callbacks.box_position is None:
            return 'box1'  # Can't find box and/or box target, so forget about pushing boxes
        else:
            self.led2_pub.publish(3)  # Off
            self.led1_pub.publish(1)  # Off
            distance_from_box_target = trig.get_distance(self.callbacks.bot_map_position,
                                                         self.callbacks.box_target_position)
            distance_from_box = trig.get_distance(self.callbacks.bot_map_position, self.callbacks.box_position)
            if distance_from_box_target < distance_from_box:
                return 'box2'
            else:
                return 'box8'


class MoveCloseToBox(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'push'])
        self.callbacks = callbacks
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty())

    def execute(self, userdata):
        global shutdown_requested

        if shutdown_requested:
            return 'done4'

        get_point_heading = trig.get_heading_between_points(self.callbacks.box_position, self.callbacks.box_target_position)

        # Move around box if bot is closer to goal than box
        box_distance_to_goal = trig.get_distance(self.callbacks.box_position, self.callbacks.box_target_position)
        bot_distance_to_goal = trig.get_distance(self.callbacks.bot_position, self.callbacks.box_target_position)
        offset = 0

        self.clear_costmap()

        if bot_distance_to_goal < box_distance_to_goal:
            offset = 0.32
            print("Going to intermediate goal")
            left_get_point_heading = get_point_heading - 90
            if left_get_point_heading < 0:
                left_get_point_heading += 360

            left_goal_position = trig.get_point(self.callbacks.box_position, 0.8, left_get_point_heading)
            right_goal_position = trig.get_point(self.callbacks.box_position, 0.8, (get_point_heading + 90) % 360)

            #print("------------------")
            #print("left")
            #print(left_goal_position)
            #print("right")
            #print(right_goal_position)
            #print("real")
            #print(self.callbacks.bot_position)
            #print("------------------")

            bot_distance_to_left = trig.get_distance(self.callbacks.bot_position, left_goal_position)
            bot_distance_to_right = trig.get_distance(self.callbacks.bot_position, right_goal_position)

            if bot_distance_to_left < bot_distance_to_right:
                print("Going to left side")
                #print(left_goal_position)
                self.goal.target_pose.header.frame_id = "map"
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = left_goal_position.x
                self.goal.target_pose.pose.position.y = left_goal_position.y
            else:
                print("Going to right side")
                #print(right_goal_position)
                self.goal.target_pose.header.frame_id = "map"
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = right_goal_position.x
                self.goal.target_pose.pose.position.y = right_goal_position.y
            #print(self.callbacks.box_position)

            goal_yaw = math.radians(90 - 180)
            goal_quaternion = quaternion_from_euler(0.0, 0.0, goal_yaw)
            #print(goal_quaternion)

            self.goal.target_pose.pose.orientation.x = goal_quaternion[0]
            self.goal.target_pose.pose.orientation.y = goal_quaternion[1]
            self.goal.target_pose.pose.orientation.z = goal_quaternion[2]
            self.goal.target_pose.pose.orientation.w = goal_quaternion[3]

            print("sending intermediate goal")
            self.client.send_goal(self.goal)
            print(self.client.wait_for_result())
            print(self.client.get_result())

            self.clear_costmap()

        # Move to proper pushing location
        goal_position = trig.get_point(self.callbacks.box_position, offset + 0.5, get_point_heading)
        goal_heading = (get_point_heading + 180) % 360

        goal_yaw = math.radians(goal_heading - 180)
        goal_quaternion = quaternion_from_euler(0.0, 0.0, goal_yaw)
        #print(goal_quaternion)

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = goal_position.x
        self.goal.target_pose.pose.position.y = goal_position.y
        self.goal.target_pose.pose.orientation.x = goal_quaternion[0]
        self.goal.target_pose.pose.orientation.y = goal_quaternion[1]
        self.goal.target_pose.pose.orientation.z = goal_quaternion[2]
        self.goal.target_pose.pose.orientation.w = goal_quaternion[3]

        if shutdown_requested:
            return 'done4'

        print("Going to main goal")
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        if shutdown_requested:
            return 'done4'
        else:
            return 'push'


class Push(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'box1', 'reverse'])
        self.callbacks = callbacks
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()

        self.led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        # self.callbacks.left_bumper_pressed
        # self.callbacks.middle_bumper_pressed
        # self.callbacks.right_bumper_pressed
        # Get the current x position, we'll move half a meter backwards
        initX = self.callbacks.bot_position.x
        currX = self.callbacks.bot_position.x
        # Timer incase we can't backup or are taking too long
        start = time.time()
        duration = 10
        # move forwards while moved < 0.2 meters and taking < 3 secs
        detected_first_press = False
        returnState = 'reverse'
        while time.time() - start < duration and not detected_first_press:
            self.move_command(0.2)
            if shutdown_requested:
                return 'done4'
            # If we hit the box, the bumper should be pressed and we keep going
            if self.callbacks.middle_bumper_pressed:
                returnState = self.push_until_fail()
                detected_first_press = True

        return returnState

    def move_command(self, speed):
        # this way I didn't have to write the 2 lines of code a million times
        target_heading = trig.get_heading_between_points(self.callbacks.box_target_position, self.callbacks.bot_position)
        error = trig.minimum_angle_between_headings(self.callbacks.bot_heading, target_heading)

        self.twist.angular.z = -error * 0.1
        self.twist.linear.x = speed
        self.cmd_vel_pub.publish(self.twist)

    def push_until_fail(self):
        global shutdown_requested
        hit_time = time.time()
        # push forever
        while time.time() - hit_time < 2:
            if self.callbacks.middle_bumper_pressed:
                hit_time = time.time()

            self.move_command(0.2)
            if shutdown_requested:
                return 'done4'

            box_position = trig.get_point(self.callbacks.bot_position, 0.405, self.callbacks.bot_heading)
            if trig.get_distance(box_position, self.callbacks.box_target_position) < 0.15:
                self.led1_pub.publish(1)  # green
                self.led2_pub.publish(3)  # red
                self.sound_pub.publish(1)
                time.sleep(5)
                self.led1_pub.publish(0)  # off
                self.led2_pub.publish(0)  # off
                return 'box1'
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        return 'reverse'


class Reverse(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'move_close_to_box', 'find_markers'])
        self.callbacks = callbacks
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()

    def execute(self, userdata):
        global shutdown_requested
        self.callbacks.box_position = None

        # Get the current x position, we'll move half a meter backwards
        initX = self.callbacks.bot_position.x
        currX = self.callbacks.bot_position.x
        # Timer incase we can't backup or are taking too long
        start = time.time()
        duration = 2
        # move backwards while moved < 0.5 meters and taking < 5 secs
        while abs(currX-initX) < 1.5 and self.callbacks.box_position is None:
            self.twist.linear.x = -0.4 #TODO 0.7
            self.cmd_vel_pub.publish(self.twist)
            currX = self.callbacks.bot_position.x

            if shutdown_requested:
                return 'done4'

        # rospy.loginfo("Time! duration: {}".format(rospy.Time.now() - time))
        return 'find_marker'


class Box1(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'box4'])
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

        # TODO: check shape

        if shutdown_requested:
            return 'done4'
        else:
            return 'box4'


class Box2(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'move_close_to_box'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

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

        if shutdown_requested:
            return 'done4'
        else:
            return 'move_close_to_box'


class Box4(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'box5'])
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

        # TODO: check shape

        if shutdown_requested:
            return 'done4'
        else:
            return 'box5'


class Box5(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'return'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

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

        # TODO: check shape

        if shutdown_requested:
            return 'done4'
        else:
            return 'return'


class Box8(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done4', 'move_close_to_box'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

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

        if shutdown_requested:
            return 'done4'
        else:
            return 'move_close_to_box'


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


def get_state_machine(callbacks):

    # Create done outcome which will stop the state machine
    sm_event_4 = smach.StateMachine(outcomes=['DONE4', 'SUCCESS4'])

    with sm_event_4:
        smach.StateMachine.add('LOCALIZE', Localize(callbacks),
                               transitions={'done4': 'DONE4', 'find_markers': 'FIND_MARKERS',
                                            'box2': 'BOX2', 'box8': 'BOX8'})

        smach.StateMachine.add('FIND_MARKERS', FindMarkers(callbacks),
                               transitions={'done4': 'DONE4', 'box1': 'BOX1', 'box2': 'BOX2', 'box8': 'BOX8'})
        smach.StateMachine.add('MOVE_CLOSE_TO_BOX', MoveCloseToBox(callbacks),
                               transitions={'done4': 'DONE4', 'push': 'PUSH'})
        smach.StateMachine.add('PUSH', Push(callbacks),
                               transitions={'done4': 'DONE4', 'box1': 'BOX1', 'reverse': 'REVERSE'})
        smach.StateMachine.add('REVERSE', Reverse(callbacks),
                               transitions={'done4': 'DONE4', 'move_close_to_box': 'MOVE_CLOSE_TO_BOX',
                                            'find_markers': 'FIND_MARKERS'})

        smach.StateMachine.add('BOX1', Box1(callbacks),
                               transitions={'done4': 'DONE4', 'box4': 'BOX4'})
        smach.StateMachine.add('BOX2', Box2(callbacks),
                               transitions={'done4': 'DONE4', 'move_close_to_box': 'MOVE_CLOSE_TO_BOX'})
        smach.StateMachine.add('BOX4', Box4(callbacks),
                               transitions={'done4': 'DONE4', 'box5': 'BOX5'})
        smach.StateMachine.add('BOX5', Box5(callbacks),
                               transitions={'done4': 'DONE4', 'return': 'RETURN'})
        smach.StateMachine.add('BOX8', Box8(callbacks),
                               transitions={'done4': 'DONE4', 'move_close_to_box': 'MOVE_CLOSE_TO_BOX'})
        smach.StateMachine.add('RETURN', Return(callbacks),
                               transitions={'success4': 'SUCCESS4', 'done4': 'DONE4'})
    return sm_event_4
