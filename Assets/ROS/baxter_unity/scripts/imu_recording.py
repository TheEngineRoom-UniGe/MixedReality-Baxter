#!/usr/bin/env python3
import rospy
import os
import sys
import argparse
import rospkg

from std_msgs.msg import Bool
from baxter_unity.msg import Imu9


class ImuRecorder:

    def __init__(self, device, folder_path):

        self.device = device
        self.folder_path = folder_path
        self.is_recording = False

        rospy.Subscriber(self.device + "/backPose", Imu9, self.callback, self.device + "_backPose")
        rospy.Subscriber(self.device + "/wristPose", Imu9, self.callback, self.device + "_wristPose")
        rospy.Subscriber("/recording", Bool, self.update_recording_status)

        rospy.loginfo("{0} IMU recorder initialized".format(self.device))


    def update_recording_status(self, msg):
        self.is_recording = msg.data
        if(self.is_recording):
            rospy.loginfo("{0} IMU recorder started logging data".format(self.device))

    def callback(self, data, topic_name):

        if self.is_recording:
            data_string = (
                str(data.header.stamp) + "," + 
                str(data.pose.orientation.x) + "," + 
                str(data.pose.orientation.y) + "," +
                str(data.pose.orientation.z) + "," + 
                str(data.pose.orientation.w) + "," + 
                str(data.linear_acceleration.x) + "," + 
                str(data.linear_acceleration.y) + "," + 
                str(data.linear_acceleration.z) + "," + 
                str(data.angular_velocity.x) + "," + 
                str(data.angular_velocity.y) + "," + 
                str(data.angular_velocity.z) + "," +  "\n")
            self.print_to_file(data_string, topic_name)


    def print_to_file (self, data_string, topic_name):
        filename = self.folder_path + "/" + topic_name + ".txt"

        f = open(filename, "a")
        f.write(data_string)
        f.close()


def main():

    rospy.init_node('imu_recording', disable_signals=True)
    
    # Parse arguments from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-d', '--device', required=True, type=str,
        help='Device name'
    )
    required.add_argument(
        '-f', '--file_name', required=True, type=str,
        help='log file name'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('baxter_unity')

    imu_folder_path = pkg_path + "/data/" + args.file_name 
    imu_recorder = ImuRecorder(args.device, imu_folder_path)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
