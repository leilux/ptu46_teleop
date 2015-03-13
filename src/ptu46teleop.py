#!/usr/bin/env python
#coding: utf-8

import roslib; roslib.load_manifest("ptu46_teleop")
import rospy
from turtlesim.msg import Velocity
from sensor_msgs.msg import JointState
from ptu46_teleop.msg import AndroidControl


class PTU46Teleop(object):
    def __init__(self):
        self._arc_p = 0.00089759763795882463

        self.pan, self.tilt = 0, 0
        self.pan_rate, self.tilt_rate = 0, 0
        self.pan_speed, self.tilt_speed = 1.5 * 0.01745, 1.5 * 0.01745
        self.pan_speed_c, self.tilt_speed_c = 20 * 0.01745, 15 * 0.01745
        self.pan_rate_amend, self.tilt_rate_amend = 5 * 0.01745, 5 * 0.01745
        self.delta = 1 * 0.01745 # pi/180.0
        self.continue_key = None

        self.max_duration = rospy.Duration(0.1) # 100ms
        self.lasttime = rospy.Time.now()

        self.sub_teleop = rospy.Subscriber(
            '/turtle1/command_velocity', 
            Velocity, 
            self.cb_teleop)

        self.sub_android_teleop = rospy.Subscriber(
            '/androidcontrol',
            AndroidControl,
            self.cb_android_teleop)

        self.sub_ptu_state = rospy.Subscriber(
            '/ptu/state',
            JointState,
            self.cb_ptu_state)

        self.pub_ptu = rospy.Publisher('/ptu/cmd', JointState)
        self.keymap = {
            (2,0): 'up',
            (-2,0): 'down',
            (0,2): 'left',
            (0,-2): 'right',}

    def cb_android_teleop(self, msg):
        key = ({1:'left', 2: 'up', 3:'right', 4: 'down'})[msg.key]
        self.cb_key(key)


    def cb_teleop(self, msg):
        key = self.keymap[(msg.linear, msg.angular)]
        self.cb_key(key)


    def cb_key(self, key):

        '''linear, angular ↑(2,0) ↓(-2,0) ←(0,2) →(0,-2)'''
        currenttime = rospy.Time.now()

        msg_out = JointState()
        msg_out.header.stamp = currenttime
        msg_out.name = ['pan', 'tilt']
        msg_out.velocity = [1.0, 0.8] # 1500 pos/s
        #msg_out.velocity = [1.347, 1.347] # 1500 pos/s

        if (currenttime - self.lasttime > self.max_duration):
            print '---'
            if   key == 'up':
                self.tilt_rate = -self.tilt_speed
                self.pan_rate = 0
            elif key == 'down':
                self.tilt_rate = self.tilt_speed
                self.pan_rate = 0
            elif key == 'right':
                self.pan_rate = -self.pan_speed
                self.tilt_rate = 0
            elif key == 'left':
                self.pan_rate = self.pan_speed
                self.tilt_rate = 0
            else:
                return
            self.continue_key = key 

        elif self.continue_key == key: # continue key press
            print '|'
            if   key == 'up':
                self.tilt_rate = -self.tilt_speed_c
            elif key == 'down':
                self.tilt_rate = self.tilt_speed_c
            elif key == 'right':
                self.pan_rate = -self.pan_speed_c
            elif key == 'left':
                self.pan_rate = self.pan_speed_c
            else:
                return
        else: # quick → ← situation
            print '<>'
            if   key == 'up' and self.continue_key == 'down':
                self.tilt_rate = -self.tilt_rate_amend
            elif key == 'down' and self.continue_key == 'up':
                self.tilt_rate = self.tilt_rate_amend
            elif key == 'right' and self.continue_key == 'left':
                self.pan_rate = -self.pan_rate_amend
            elif key == 'left' and self.continue_key == 'right':
                self.pan_rate = self.pan_rate_amend
            else:
                return
            self.continue_key = key

        msg_out.position = [
            self.pan + self.pan_rate,
            self.tilt + self.tilt_rate
        ]
        self.pub_ptu.publish(msg_out)
        self.lasttime = currenttime


    def cb_ptu_state(self, msg):
        '''name[], position[], velocity[0.898, 0.898]'''
        self.pan, self.tilt = msg.position


if __name__ == "__main__":
    rospy.init_node("ptu46teleop")
    PTU46Teleop()
    rospy.spin()
