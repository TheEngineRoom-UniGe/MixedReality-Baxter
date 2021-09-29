import rospy
import time
import argparse
import sys
import copy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from panda_unity.msg import ArmJoints, ArmTrajectory
from panda_unity.srv import PlannerService, PlannerServiceResponse

class MotionPlanner:

  def __init__(self):
  	  	
    group_name = "panda_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)
    self.home_joint_config = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

  # Plane trajectory in cartesian space
  def plan_cartesian_trajectory(self, destination_pose, start_joint_angles):

    current_joint_state = JointState()
    joint_names = ["panda_joint" + str(joint) for joint in [1,2,3,4,5,6,7]]
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    self.move_group.set_start_state(moveit_robot_state)
    self.move_group.set_goal_tolerance(10e-3)

    (plan, fraction) = self.move_group.compute_cartesian_path([destination_pose], 0.1, 0.0)

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, start_joint_angles)
        exit(1)

    return plan

  # Plan trajectory in joint space
  def plan_return_to_home(self, final_joint_config, start_joint_config):

    current_joint_state = JointState()
    joint_names = ["panda_joint" + str(joint) for joint in [1,2,3,4,5,6,7]]
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
        """.format(destination_pose, start_joint_angles)
        exit(1)

    return plan[1]

  def routine_dispatcher(self, req):

      if req.routine == "pickandplace":
          return pick_and_place(req)
      elif req.routine == "gohome":
          return go_home(req)
      else return None
    
  # Plan pick and place action
  def pick_and_place(self, req):

    response = PlannerServiceResponse()

    # Initial joint configuration
    current_joint_states = req.joints.angles
    initial_joint_configuration = copy.deepcopy(current_joint_states)
    
    # Pre grasp - position gripper directly above target object
    pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, initial_joint_configuration)

    previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
    response.arm_trajectory.trajectory.append(pre_grasp_traj)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.1
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
    place_pose.position.z -= 0.1*0.8
    place_traj = self.plan_cartesian_trajectory(place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
    response.arm_trajectory.trajectory.append(place_traj)

    # Lift-up
    lift_up_pose = copy.deepcopy(req.place_pose)
    lift_up_pose.position.z += 0.2
    lift_up_traj = self.plan_cartesian_trajectory(lift_up_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = lift_up_traj.joint_trajectory.points[-1].positions
    response.arm_trajectory.trajectory.append(lift_up_traj)
        
    self.move_group.clear_pose_targets()
  
    return response

  # Plan go home action
  def go_home(self, req):

    response = PlannerServiceResponse()
  
    # Return to home pose
    return_home_traj = self.plan_return_to_home(self.home_joint_config, req.joints.angles)
    response.arm_trajectory.trajectory.append(return_home_traj)

    self.move_group.clear_pose_targets()
  
    return response


def main():

  # Initialize node
  rospy.init_node('motion_planner_service_node')
  moveit_commander.roscpp_initialize(sys.argv) 

  # Define motion planner object
  motion_planner = MotionPlanner()
  s = rospy.Service('motion_planner_service', PlannerService, motion_planner.routine_dispatcher)
      
  print("Ready to plan")
  rospy.spin()


if __name__ == "__main__":
    main()
    


