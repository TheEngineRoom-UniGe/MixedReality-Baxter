#! /usr/bin/python

import rospy
import rospkg
import argparse
import cv2
import os

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

# Logger class
class Logger():

    def __init__(self, logfilename):
        self.f = open(logfilename, "w")

    def log(self, msg):
        self.f.write("{0},{1}\n".format(rospy.Time.now(), msg))

    def close(self):
        self.f.close()
        print("Log file successfully closed")


class FrameSaver():

    def __init__(self, folder_path, log_path):
        self.count = 1
        self.frames_folder_path = folder_path
        self.frames_log_path = log_path
        self.bridge = CvBridge()
        self.frame_logger = Logger(log_path)
        self.recording = False

    def handle_command(self, msg):
        if msg.data == True:
            rospy.loginfo("Started recording frames")
            self.recording = True
        else:
            self.recording = False
            self.frame_logger.close()
            rospy.signal_shutdown("Stopped recording frames")


    def save_frame(self, msg):
        if self.recording:
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                frame_path = self.frames_folder_path + '/frame_' + str(self.count) + '.jpeg'
                self.frame_logger.log(frame_path)
                cv2.imwrite(frame_path, cv2_img)
                self.count += 1


def main():

    rospy.init_node('data_saver')

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file_name', required=True, type=str,
        help='log file name'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('baxter_unity')

    frames_folder_path = pkg_path + "/data/" + args.file_name + "/frames" 
    frames_log_path = pkg_path + "/data/" + args.file_name + "/frames_log.txt"

    rospy.sleep(1)

    fs = FrameSaver(frames_folder_path, frames_log_path)

    rospy.Subscriber("/usb_cam/image_raw", Image, fs.save_frame)
    rospy.Subscriber("/recording", Bool, fs.handle_command)
    
    rospy.loginfo("Waiting for user input to start recording frames ...")
    rospy.spin()

if __name__ == '__main__':
    main()
