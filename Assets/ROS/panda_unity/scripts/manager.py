#!/usr/bin/env python

import rospy
import time
import logging
import rospkg
import argparse
from std_msgs.msg import String, Bool, Int32

class Manager():

    def __init__(self):
        self.publisher = rospy.Publisher('next_step', Int32, queue_size=10)
        self.time = time.time()
        self.step_count = 0
        self.reset_count = 0

    def spin(self):
        print("Listening...")
        while not rospy.is_shutdown():
            x = int(input())
            int_msg = Int32()
            int_msg.data = x
            self.publisher.publish(int_msg)

            if x in [1,2,3,4]:
                self.time = time.time()
            elif x == 0:
                logging.info("Reset count: " + str(self.reset_count))
                logging.info("END TRIAL")
                self.reset_count = 0
                self.step_count = 0
            elif x == -1:
                self.reset_count += 1
            elif(x == 8):
                logging.info("START TRIAL WITH HOLO")
            elif(x == 9):
                logging.info("START TRIAL WITHOUT HOLO")

    def step_done_callback(self, msg):
        self.step_count += 1
        elapsed = time.time() - self.time
        logging.info("Step " + str(self.step_count) + ": " + str(elapsed))

    def collision_count_callback(self, msg):
        logging.info("Collisions: " + str(msg.data))


def main():

    rospy.init_node('manager', disable_signals=True, log_level=rospy.WARN)

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

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('panda_unity')

    logging.basicConfig(
        filename = pkg_path + "/data/" + log_file + ".log",
        filemode = 'a',
        format = '%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
        datefmt = '%H:%M:%S',
        level = logging.INFO,
        force = True
    )

    manager = Manager()
    rospy.Subscriber("step_done", Bool, manager.step_done_callback)
    rospy.Subscriber("collision_count", Int32, manager.collision_count_callback)
    
    time.sleep(2.0)

    manager.spin()

    
if __name__ == "__main__":
    main()
    

    
