#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from panda_unity.srv import PlannerService
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Bool


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True, log_level=rospy.WARN)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        'motion_planner_service': RosService('motion_planner_service', PlannerService),
        'next_step': RosSubscriber('next_step', Int32, tcp_server),
        'step_done': RosPublisher('step_done', Bool),
        'collision_count': RosPublisher('collision_count', Int32)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
