#!/usr/bin/env python
from __future__ import print_function

import rospy
import rospkg
import cv2
import serial
import threading
import time
import argparse

from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from baxter_unity.msg import PlannedTrajectory, NextAction

from cv_bridge import CvBridge

# Logger class
class Logger():

    def __init__(self, logfilename):
        self.f = open(logfilename, "a")

    def log(self, msg):
        self.f.write("{0}: {1}\n".format(time.ctime(time.time()), msg))

    def close(self):
        self.f.close()


# Thread class to listen to serial port for user input
class SerialReaderTask:

    def __init__(self):
        self.running = True
        self.is_paused = True
        self.t_received_command = 0.0

    def handle_data(self, data):
        if int(data)== 1 and abs(time.time() - self.t_received_command) > 5.0:
            self.is_paused = not self.is_paused
            self.t_received_command = time.time()
            print("Next action is paused:", self.is_paused)

    def stop(self):
        self.running = False

    def run(self, ser):
        while self.running:
            reading = ser.read(1).decode()
            self.handle_data(reading)
            

# Plan manager class to read and publish robot's actions
class PlanManager():

    def __init__(self, pkg_path_name, log_file_name):
        rospy.Subscriber("/action_done", Bool, self.next_action_handler)
        self.next_action_pub = rospy.Publisher('/next_action', NextAction, queue_size=10)
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=10)

        # Start internal task to monitor serial input
        self.serial_port = serial.Serial('/dev/ttyACM0')
        self.reader_task = SerialReaderTask()
        self.thread = threading.Thread(target=self.reader_task.run, args=(self.serial_port,))
        self.thread.start()

        # Internal logger
        self.logger = Logger(pkg_path_name + "/logs/" + log_file_name + ".txt")
        self.logger.log("------------------------")

        # Variables to keep track of time for each action and robot idle time
        self.task_time = 0.0
        self.pause_time = 0.0

        # Read plan steps from file
        with open(pkg_path_name + "/data/plan.txt") as f:
            self.plan_steps = f.readlines()
            
        self.plan_length = len(self.plan_steps)
        self.action_idx = 0

        rospy.sleep(2.0)
        print("Ready to accept user inputs ...")

        # Publish start screen image on robot's display
        self.images_path = pkg_path_name + "/images/"
        img = cv2.imread(self.images_path + "start.png")
        img_msg = CvBridge().cv2_to_imgmsg(img)
        rospy.sleep(0.5)
        self.image_pub.publish(img_msg)
        
    # Handles next action based on paused status
    def next_action_handler(self, msg):
        if(not self.reader_task.is_paused):
            self.publish_next(msg)
        # If paused, wait until unpausing before publishing
        else:
            self.publish_next_later(msg)

    # Immediately publish next action(s)
    def publish_next(self, msg):
        # Publish next action(s)
        if self.action_idx < self.plan_length:

            # Store initial time
            if(self.action_idx == 0):
                self.task_time = time.time()

            instruction = self.plan_steps[self.action_idx].split()
            next_action_msg = NextAction()
            # If row contains two instructions, plan for both arms
            if(len(instruction) > 2):
                next_action_msg.op = [instruction[0], instruction[2]]
                next_action_msg.id = [int(instruction[1]), int(instruction[3])]
            # Else, plan for single arm
            else:
                next_action_msg.op = [instruction[0]]
                next_action_msg.id = [int(instruction[1])]

            self.next_action_pub.publish(next_action_msg)

            # If certain step of plan is reached, publish new image with instructions
            img = cv2.imread(self.images_path + "step" + str(self.action_idx) + ".png")
            #img_resized = cv2.resize(img, (1000, 600), interpolation = cv2.INTER_AREA)
            img_msg = CvBridge().cv2_to_imgmsg(img)
            self.image_pub.publish(img_msg)

            self.action_idx += 1

        # If last "action_done" message, save total task time and exit
        elif self.action_idx == self.plan_length:
            elapsed_task_time = time.time() - self.task_time
            self.logger.log("Total Task Time: {0} seconds".format(elapsed_task_time))

            # Publish end screen image to display at the end of planning steps
            img = cv2.imread(self.images_path + "end.png")
            img_msg = CvBridge().cv2_to_imgmsg(img)
            self.image_pub.publish(img_msg)

            # Stop thread process, close log file and exit
            self.reader_task.stop()
            self.thread.join()
            self.logger.close()
            rospy.sleep(0.5)
            rospy.signal_shutdown("Plan finished")

    # Wait for unpausing before publishing next action(s)
    def publish_next_later(self, msg):
        self.pause_time = time.time()
        while(self.reader_task.is_paused):
            try:
                rospy.sleep(0.2)
            except KeyboardInterrupt:
                self.reader_task.stop()
                self.thread.join()
                self.logger.close()
                rospy.sleep(0.5)
                rospy.signal_shutdown("Process shutdown")

        # If not first action, keep track of robot idle time during the pause
        if(self.action_idx != 0):
            elapsed_pause_time = time.time() - self.pause_time
            self.logger.log("Action {0} - Robot Idle Time: {1} seconds".format(self.action_idx, elapsed_pause_time))

        self.publish_next(msg)


def main():

    rospy.init_node('plan_manager', disable_signals=True, log_level=rospy.FATAL)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('baxter_unity')

    # Parse arguments from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file_name', required=True, type=str,
        help='log file name'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    log_file = args.file_name

    # Instantiate plan manager object
    manager = PlanManager(pkg_path, log_file)

    rospy.spin()
   
           
if __name__ == "__main__":
    main()
    

    
