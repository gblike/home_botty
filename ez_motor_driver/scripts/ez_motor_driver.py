#!/usr/bin/env python

import rospy
import time
import serial
import sys
import Queue as Q
from multiprocessing import Process, Queue
import io
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

class Connection():
    def __init__(self):
        self.actions = []
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.message_queue = Queue(maxsize=1)
        if(not self.ser.isOpen()):
            self.ser.open()
        time.sleep(2)

    def update_velocity(self, velocity):
        print(str(velocity))
        directions = ('STOP', 'STOP')
        if (velocity.linear.x > 0 and velocity.angular.z == 0):
            directions = ('FORWARD', 'FORWARD')
        elif (velocity.linear.x < 0 and velocity.angular.z == 0):
            directions = ('BACKWARD', 'BACKWARD')
        elif (velocity.linear.x == 0 and velocity.angular.z > 0): # Turn right
            directions = ('FORWARD', 'BACKWARD')
        elif (velocity.linear.x == 0 and velocity.angular.z < 0): # Turn LEFT
            directions = ('BACKWARD', 'FORWARD')

        try:
            self.message_queue.put(zip(["LEFT","RIGHT"], directions), False)
        except Q.Full:
            pass

    def pose_callback(self, pose):
        print(str(pose))
        #serial.write('a')
        #serial.flush(

    def connect(self):
        pass

    def run(self):
        rospy.init_node('ezmotor', anonymous=True)

        # place to receive new goal
        cmd_vel_topic = '/cmd_vel'
        self.sub = rospy.Subscriber(cmd_vel_topic,  Twist, self.update_velocity)

        # place to receive feedback
        #odometry_topic = '/rtabmap/localization_pose'
        #self.sub = rospy.Subscriber(odometry_topic, PoseWithCovarianceStamped, self.pose_callback)
        listener = tf.TransformListener()

        print("Initializing")
        start = 0
        def read_input(serial_stream):
            while not rospy.is_shutdown():
                time.sleep(0.1)
                if serial_stream.inWaiting() > 0:
                    data_str = serial_stream.read(serial_stream.inWaiting())
                    sys.stdout.write(data_str)

        def stream_commands(message_queue, serial_stream):
            while not rospy.is_shutdown():
                time.sleep(0.25)
                if message_queue.qsize() > 0:
                    message = message_queue.get()
                    for motor, direction in message:
                        cmd = " ".join([motor, direction, str(76)]) + "\n"
                        print(cmd)
                        if(self.ser.isOpen()):
                            self.ser.write(cmd)
                            self.ser.flush()
                        else:
                            print("serial connection is closed")

        def ros_status():
            while not rospy.is_shutdown():
                time.sleep(0.1)


        read_loop = Process(target=read_input, args=(self.ser,))
        read_loop.daemon = True

        write_loop = Process(target=stream_commands, args=(self.message_queue, self.ser,))
        write_loop.daemon = True

        ros_client = Process(target=ros_status)
        ros_client.daemon = True

        ros_client.start()
        read_loop.start()
        write_loop.start()
        ros_client.join()


if __name__ == '__main__':
    serial_proxy = Connection()
    serial_proxy.connect()
    serial_proxy.run()
