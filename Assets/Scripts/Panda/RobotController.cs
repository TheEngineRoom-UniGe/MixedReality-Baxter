using System.Collections;
using System.Linq;

using RosMessageTypes.PandaUnity;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;

public class RobotController : MonoBehaviour
{
    private int numRobotJoints = 7;

    // Timing variables for rendering trajectory
    private float jointAssignmentWaitRest = 0.01f;
    private float jointAssignmentWait = 0.005f;

    // Offset variables for picking and placing objects
    private Vector3 liftOffset = 0.2f * Vector3.up;
    private Vector3 placeOffset = 0.04f * Vector3.up;

    // Scene objects (robot and interactables)
    private GameObject robot;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private ArticulationBody[] gripper;

    // Utility variables
    public bool coroutineRunning = false;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
        Return
    };

    
    public void Init(GameObject robot, Transform ground)
    {
        this.robot = robot;

        GetRobotReference();
        GoToRestPosition();

    }

    void GetRobotReference()
    {
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string link1 = "panda_link0/panda_link1";
        jointArticulationBodies[0] = robot.transform.Find(link1).GetComponent<ArticulationBody>();

        string link2 = link1 + "/panda_link2";
        jointArticulationBodies[1] = robot.transform.Find(link2).GetComponent<ArticulationBody>();

        string link3 = link2 + "/panda_link3";
        jointArticulationBodies[2] = robot.transform.Find(link3).GetComponent<ArticulationBody>();

        string link4 = link3 + "/panda_link4";
        jointArticulationBodies[3] = robot.transform.Find(link4).GetComponent<ArticulationBody>();

        string link5 = link4 + "/panda_link5";
        jointArticulationBodies[4] = robot.transform.Find(link5).GetComponent<ArticulationBody>();

        string link6 = link5 + "/panda_link6";
        jointArticulationBodies[5] = robot.transform.Find(link6).GetComponent<ArticulationBody>();

        string link7 = link6 + "/panda_link7";
        jointArticulationBodies[6] = robot.transform.Find(link7).GetComponent<ArticulationBody>();

        string hand = link7 + "/panda_link8/panda_hand";
        // Find left and right fingers
        string left_gripper = hand + "/panda_leftfinger";
        string right_gripper = hand + "/panda_rightfinger";

        gripper = new ArticulationBody[2];
        gripper[0] = robot.transform.Find(left_gripper).GetComponent<ArticulationBody>();
        gripper[1] = robot.transform.Find(right_gripper).GetComponent<ArticulationBody>();

    }

    private void CloseGripper()
    {
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.0f;
        rightDrive.target = 0.0f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    private void OpenGripper()
    {
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.04f;
        rightDrive.target = 0.04f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    ArmJoints InitialJointConfig()
    {
        ArmJoints joints = new ArmJoints();

        joints.angles = new double[numRobotJoints];
        for (int i = 0; i < numRobotJoints; i++)
        {
            joints.angles[i] = Mathf.Deg2Rad * jointArticulationBodies[i].xDrive.target;
        }
        return joints;
    }

    public void GoToRestPosition()
    {
        StartCoroutine(GoToRest());
    }

    private IEnumerator GoToRest()
    {
        float[] restPosition = { 0.0f, -1.0472f, 0.0f, -2.617f, 0.0f, 1.571f, 0.785f };
        float[] target = new float[numRobotJoints];

        for (int i = 0; i < restPosition.Length; i++)
        {
            target[i] = Mathf.Rad2Deg * (float)restPosition[i];
        }
        var steps = 100;
        for (int i = 0; i <= steps; i++)
        {
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = target[joint] * (float)(1.0f / steps) * (float)i;
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWaitRest);
        }
        OpenGripper();
    }

    // Pick and place service request
    public PlannerServiceRequest PickAndPlaceService(int idx, Vector3 pickPosition, Vector3 placePosition)
    {
        coroutineRunning = true;

        PlannerServiceRequest request = new PlannerServiceRequest();
        request.routine = "pickandplace";

        // Initial Joint Config
        request.joints = InitialJointConfig();

        // Pick Pose
        ROSGeometry.Quaternion<FLU> fixedOrientation;
        if (idx == 0)
        {
            fixedOrientation = Quaternion.Euler(180, -135, 0).To<FLU>();
        }
        else
        {
            fixedOrientation = Quaternion.Euler(180, -225, 0).To<FLU>();
        }
         
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition + liftOffset).To<FLU>(),
            orientation = fixedOrientation
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition + liftOffset).To<FLU>(),
            orientation = fixedOrientation
        };

        return request;
    }

    public PlannerServiceRequest GoHomeService()
    {
        coroutineRunning = true;

        PlannerServiceRequest request = new PlannerServiceRequest();
        request.routine = "gohome";

        // Initial Joint Config
        request.joints = InitialJointConfig();

        return request;
    }

    public void ROSServiceResponse(PlannerServiceResponse response)
    {
        if (response.arm_trajectory.trajectory.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.Log("No trajectory returned from MoveIt.");
        }
    }

    public IEnumerator ExecuteTrajectories(PlannerServiceResponse response)
    {
        if (response.arm_trajectory.trajectory != null)
        {
            var initialJointConfig = InitialJointConfig();
            double[] lastJointState = initialJointConfig.angles.Select(x => x * Mathf.Rad2Deg).ToArray();

            // For every trajectory plan returned
            //int steps = 15; // For speedup execution on HoloLens
            int steps = 30; // For execution on Editor 
            for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
            {
                if (poseIndex == response.arm_trajectory.trajectory.Length - 1)
                {
                    coroutineRunning = false;
                }

                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();
                    for (int i = 0; i <= steps; i++)
                    {
                        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                        {
                            var joint1XDrive = jointArticulationBodies[joint].xDrive;
                            joint1XDrive.target = (float)(lastJointState[joint] + (result[joint] - lastJointState[joint]) * (1.0f / steps) * i);
                            jointArticulationBodies[joint].xDrive = joint1XDrive;
                        }

                        yield return new WaitForSeconds(jointAssignmentWait);

                    }
                    // Wait for robot to achieve pose for all joint assignments
                    lastJointState = result;
                }
                // Make sure gripper is open at the beginning
                if (poseIndex == (int)Poses.PreGrasp || poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(0.5f);
                    OpenGripper();
                }
                // Close gripper on object grasping
                if (poseIndex == (int)Poses.Grasp)
                {
                    yield return new WaitForSeconds(0.5f);
                    CloseGripper();
                }
            }
        }
        
    }

    public void Spawn(Vector3 spawnPosition)
    {
        robot.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0,-180.0f,0));
        robot.SetActive(true);
    }

}