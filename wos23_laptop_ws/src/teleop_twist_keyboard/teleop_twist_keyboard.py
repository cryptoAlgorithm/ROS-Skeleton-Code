#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty


VELOCITY_STEP = 0.1

msg = """
 /-----------------\\
 |  Q  |  W  |  E  |
 |-----|-----|-----|
 |  A  |  S  |  D  |
 \\-----------------/

Q = Twist left
E = Twist right
W = Forwards
S = Backwards
A = Increase angular velocity
D = Decrease angular velocity

CTRL-C to quit
"""


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.lin = self.ang = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, lin: float, ang: float):
        '''
        Publish an updated set of linear and angular velocities
        '''
        self.condition.acquire()
        self.lin = lin
        self.ang = ang
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0)
        self.join() # Wait till this thread terimnates

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.lin
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.ang

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "Velocities:\tlinear %.1f\tangular %.1f" % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    # Linear and angular velocity
    speed = lin = ang = 0.0
    # speed = rospy.get_param("~speed", 0.0)
    # turn = rospy.get_param("~turn", 0.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(lin, ang)

        print(msg)
        while(1):
            key = getKey(key_timeout)

            if key in ['w', 's']:
                print('Forward' if key == 'w' else 'Backward')
                if lin != speed:
                    lin = speed
                    ang = 0
                else:
                    speed += VELOCITY_STEP if key == 'w' else -VELOCITY_STEP
                    lin = speed
            elif key == 'a': # Rotate wheels the opposite direction of each other such that there is minimal linear motion
                print('In place left')
                lin = 0
                ang += VELOCITY_STEP
            elif key == 'd':
                print('In place right')
                lin = 0
                ang -= VELOCITY_STEP
            elif key == 'q':
                print('Increase angular velo')
                ang += VELOCITY_STEP
            elif key == 'e':
                print('Decrease angular velo')
                ang -= VELOCITY_STEP
            else:
                lin = speed = ang = 0
                print('Stop')
                if key == '\x03': # Ctrl-C
                    print('Interrupted, exiting...')
                    break

            print(vels(lin, ang))

            pub_thread.update(lin, ang)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
