#!/usr/bin/env python

import rospy
import time
import serial
import threading
import io
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

class Connection():
    def __init__(self):
        self.actions = []

    def update_velocity(self, velocity):
        print(str(velocity))

    def pose_callback(self, pose):
        print(str(pose))
        #serial.write('a')
        #serial.flush(
        

    def connect(self):
        pass
    def write_message(self,motor, direction, speed):
        out = bytearray()
        if motor == 'RIGHT':
            out.append('R')
        else:
            out.append('L')

        if direction == 'FORWARD':
            out.append('F')
        elif direction == 'BACKWARD':
            out.append('B')
        else:
            out.append('S')

        out.append(speed)

        return out

    def run(self):
        rospy.init_node('ezmotor', anonymous=True)

        # place to receive new goal
        cmd_vel_topic = '/ezmotor/cmd_vel'
        #self.sub = rospy.Subscriber(cmd_vel_topic,  Twist, self.update_velocity)

        # place to receive feedback
        odometry_topic = '/rtabmap/localization_pose'
        #self.sub = rospy.Subscriber(odometry_topic, PoseWithCovarianceStamped, self.pose_callback)
        #listener = tf.TransformListener()
        refresh = rospy.Rate(1)


        commands = [Connection.DRIVE_FORWARD, Connection.SPIN_CLOCKWISE, Connection.SPIN_COUNTER_CLOCKWISE, Connection.DRIVE_BACKWARDS, Connection.RELAX_MOTORS]
        print("Initializing")
        ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)
        start = 0
        while not rospy.is_shutdown():
            try:
                #(trans, rot) = listener.lookupTransform('/map', 'camera_link', rospy.Time(0))
                #print(str((trans,rot)))
                currenttime = int(round(time.time() * 1000))
                m = self.write_message('LEFT', 'FORWARD', start)
                ser.write(m)
                m = self.write_message('RIGHT', 'BACKWARD', 255)
                ser.write(m)
                time.sleep(0.1)
                if ser.inWaiting() > 0:
                    data_str = ser.read(ser.inWaiting())
                    print(data_str)
                start = (start + 1) % 255
                print("Speed: " + str(start))
                refresh.sleep()
            except Exception, e: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print(e)
                raise SystemExit

if __name__ == '__main__':
    try:
        serial_proxy = Connection()
        serial_proxy.connect()
        serial_proxy.run()
    except rospy.ROSInterruptException: pass
