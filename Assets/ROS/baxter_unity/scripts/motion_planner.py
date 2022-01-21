#!/usr/bin/env python

from __future__ import print_function
from os import ST_SYNCHRONOUS

import rospy
import argparse
import sys
import copy
import math
import moveit_commander
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import String
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose, PoseStamped

from baxter_unity.msg import PlannedAction
from baxter_unity.srv import ActionService, ActionServiceRequest, ActionServiceResponse


class MotionPlanner:

    def __init__(self, limb, offset, obstacle):
        self.limb = limb
        self.height_offset = offset
        self.obstacle = obstacle
        self.pick_seq = 0
        self.tool_seq = 0

        self.scene = moveit_commander.PlanningSceneInterface(synchronous = True)

        group_name = self.limb + "_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.publisher = rospy.Publisher('baxter_moveit_trajectory', PlannedAction, queue_size=10)

    def log_msg(self, msg):
        print(rospy.get_name() + ": " + msg)

    def add_obstacles(self):
        self.log_msg("Adding obstacles to scene..")
    
        p = PoseStamped()
        p.header.frame_id = "world"
        p.pose.position.z = 0.4
        p.pose.orientation.w = 1
        box_name = "table"
        self.scene.add_box(box_name, p, (2, 2, self.obstacle))
        if(self.wait_for_state_update(box_name, box_is_known=True)):
            self.log_msg(box_name + " added to obstacles")
        else:
            self.log_msg("Error while adding obstacles to planning scene")

        p.header.frame_id = "world"
        p.pose.position.x = 0.7
        p.pose.position.y = -0.8
        p.pose.position.z = 0.9
        p.pose.orientation.w = 1
        box_name = "bookshelf_side_front"
        self.scene.add_box(box_name, p, (0.01, 0.2, 1.8))
        if(self.wait_for_state_update(box_name, box_is_known=True)):
            self.log_msg(box_name + " added to obstacles")
        else:
            self.log_msg("Error while adding obstacles to planning scene")

        p.header.frame_id = "world"
        p.pose.position.x = -0.1
        p.pose.position.y = -0.8
        p.pose.position.z = 0.9
        p.pose.orientation.w = 1
        box_name = "bookshelf_side_back"
        self.scene.add_box(box_name, p, (0.01, 0.2, 1.8))
        if(self.wait_for_state_update(box_name, box_is_known=True)):
            self.log_msg(box_name + " added to obstacles")
        else:
            self.log_msg("Error while adding obstacles to planning scene")

        p.header.frame_id = "world"
        p.pose.position.x = 0.1
        p.pose.position.y = -0.85
        p.pose.position.z = 1.7
        p.pose.orientation.w = 1
        box_name = "bookshelf_top"
        self.scene.add_box(box_name, p, (0.6, 0.4, 0.01))
        if(self.wait_for_state_update(box_name, box_is_known=True)):
            self.log_msg(box_name + " added to obstacles")
        else:
            self.log_msg("Error while adding obstacles to planning scene")

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=10):

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        
    # Plan straight line trajectory
    def plan_cartesian_trajectory(self, destination_pose, start_joint_angles):
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_goal_tolerance(10e-3)
        (plan, fraction) = self.move_group.compute_cartesian_path([destination_pose], 0.025, 0.0)

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan

    # Plan trajectory to given pose
    def plan_to_pose(self, destination_pose, start_joint_angles):
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_pose_target(destination_pose)
        self.move_group.set_goal_tolerance(10e-3)
        plan = self.move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, start_joint_angles)
            exit(1)

        return plan[1]
    
    # Plan joint space trajectory
    def plan_return_to_home(self, final_joint_config, start_joint_config):
        current_joint_state = JointState()
        joint_names = [self.limb + '_' + joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_config

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_joint_value_target(final_joint_config)
        self.move_group.set_goal_tolerance(10e-3)
        
        plan = self.move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(final_joint_config, start_joint_config)
            exit(1)

        return plan[1]

    # Dispatcher callback handling different actions upon request
    def dispatcher(self, req):
        if(req.action == "pick_and_place" or req.action == "put_back"):
            return self.pick_and_place(req)
        elif(req.action == "tool_handover"):
            return self.tool_handover(req)
        elif(req.action == "component_handover"):
            return self.component_handover(req)
        else:
            return None

    # Plan pick and place action
    def pick_and_place(self, req):
        op = req.action

        response = ActionServiceResponse()
        response.action = op
        response.arm_trajectory.arm = self.limb
        response.pick_seq = self.pick_seq
        self.pick_seq += 1

        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_traj = self.plan_cartesian_trajectory(req.pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = pick_up_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(pick_up_traj)

        # Move gripper to desired placement position
        move_traj = self.plan_cartesian_trajectory(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = move_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(move_traj)

        # Place - Descend and leave object in the desired position
        place_pose = copy.deepcopy(req.place_pose)
        place_pose.position.z -= self.height_offset*0.8
        place_traj = self.plan_cartesian_trajectory(place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(place_traj)
            
        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.arm_trajectory.trajectory.append(return_home_traj)

        self.move_group.clear_pose_targets()

        action_msg = PlannedAction()
        action_msg.action = op
        action_msg.arm_trajectory.arm = self.limb
        action_msg.arm_trajectory.trajectory = response.arm_trajectory.trajectory
        self.publisher.publish(action_msg)
        
        return response
  
    # Plan tool handover action
    def tool_handover(self, req):
        op = req.action
        
        response = ActionServiceResponse()
        response.action = op
        response.arm_trajectory.arm = self.limb
        response.tool_seq = self.tool_seq
        self.tool_seq += 1

        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        lift_pose = copy.deepcopy(req.pick_pose)
        lift_pose.position.z += self.height_offset  # HARDCODED VALUE FOR NOW
        lift_up_traj = self.plan_cartesian_trajectory(lift_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(lift_up_traj)
        
        # Handover - move gripper to desired handover position
        handover_traj = self.plan_to_pose(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = handover_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(handover_traj)
        
        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.arm_trajectory.trajectory.append(return_home_traj)

        self.move_group.clear_pose_targets()
        
        action_msg = PlannedAction()
        action_msg.action = op
        action_msg.arm_trajectory.arm = self.limb
        action_msg.arm_trajectory.trajectory = response.arm_trajectory.trajectory
        self.publisher.publish(action_msg)

        return response

    # Plan Component handover action
    def component_handover(self, req):
        op = req.action

        response = ActionServiceResponse()
        response.action = op
        response.arm_trajectory.arm = self.limb

        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints.angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_pick_pose = copy.deepcopy(req.pick_pose)
        pre_pick_pose.position.z -= self.height_offset * 0.5
        pre_grasp_traj = self.plan_cartesian_trajectory(pre_pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset * 0.9
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        lift_pose = copy.deepcopy(req.pick_pose)
        lift_pose.position.z -= self.height_offset * 0.5
        lift_up_traj = self.plan_cartesian_trajectory(lift_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(lift_up_traj)

        # Handover - move gripper to desired handover position
        handover_traj = self.plan_cartesian_trajectory(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = handover_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(handover_traj)

        # Put back - take object back to its original pose
        put_back_pose = copy.deepcopy(req.pick_pose)
        put_back_pose.position.z -= 0.8*self.height_offset
        put_back_traj = self.plan_cartesian_trajectory(put_back_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = put_back_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(put_back_traj)

        # Move away from obstacles
        move_away_pose = copy.deepcopy(req.pick_pose)
        move_away_pose.position.y += 0.3
        move_away_traj = self.plan_cartesian_trajectory(move_away_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = move_away_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(move_away_traj)

        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.arm_trajectory.trajectory.append(return_home_traj)

        self.move_group.clear_pose_targets()

        action_msg = PlannedAction()
        action_msg.action = op
        action_msg.arm_trajectory.arm = self.limb
        action_msg.arm_trajectory.trajectory = response.arm_trajectory.trajectory
        self.publisher.publish(action_msg)

        return response
   
   
def main():
    # Initialize node
    rospy.init_node('motion_planner_service_node', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv) 

    # Parse argument from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, type=str,
        help='limb parameter [left, right]'
    )
    required.add_argument(
        '-o', '--offset', required=True, type=float,
        help='height offset value'
    )
    required.add_argument(
        '-t', '--obstacle', required=True, type=float,
        help='obstacle height offset value'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Parsed arguments
    limb = args.limb
    offset = args.offset
    obstacle = args.obstacle

    # Define motion planner object with argument parsed
    motion_planner = MotionPlanner(limb, offset, obstacle)
    motion_planner.add_obstacles()
    s = rospy.Service('/' + limb + '_group/baxter_unity_motion_planner', ActionService, motion_planner.dispatcher)
 
    motion_planner.log_msg("Ready to plan")

    rospy.spin()


if __name__ == "__main__":
    main()
