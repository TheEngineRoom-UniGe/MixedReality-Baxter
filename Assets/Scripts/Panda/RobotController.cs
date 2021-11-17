using System.Collections;
using System.Linq;

using RosMessageTypes.PandaUnityTest;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.Visualizations;
using Unity.Robotics.UrdfImporter;

using Quaternion = UnityEngine.Quaternion;
using JointState = RosMessageTypes.Sensor.JointStateMsg;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;

public class RobotController : MonoBehaviour
{
    private int numRobotJoints = 7;

    // Timing variables for rendering trajectory
    private float jointAssignmentWaitRest = 0.01f;

    // Offset variables for picking and placing objects
    private Vector3 liftOffset = 0.2f * Vector3.up;

    // Scene objects (robot and interactables)
    private GameObject robot;
    private UrdfRobot urdfRobot;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private ArticulationBody[] gripper;

    // Drawing components
    private RobotVisualization robotVisualization;
    private Drawing3d drawing3;
    private JointState jointState;
    private Material mat;

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


    public void Init(GameObject robot, UrdfRobot urdfRobot, Transform ground, Material mat)
    {
        this.robot = robot;
        this.urdfRobot = urdfRobot;
        this.mat = mat;

        GetRobotReference();
        GoToRestPosition();

        robotVisualization = new RobotVisualization(this.urdfRobot);

        drawing3 = Drawing3d.Create(10.0f, this.mat);
        jointState = new JointState();
        string[] names = {
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2"
        };
        jointState.name = names;

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

    ArmJointsMsg InitialJointConfig()
    {
        ArmJointsMsg joints = new ArmJointsMsg();

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
    public PlannerServiceRequest PickAndPlaceService(Vector3 pickPosition, Vector3 placePosition)
    {
        coroutineRunning = true;

        PlannerServiceRequest request = new PlannerServiceRequest();
        request.routine = "pickandplace";

        // Initial Joint Config
        request.joints = InitialJointConfig();

        // Pick Pose
        Quaternion fixedOrientation = Quaternion.Euler(180, -135, 0);

        request.pick_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = (pickPosition + liftOffset).To<FLU>(),
            orientation = fixedOrientation.To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = (placePosition + liftOffset).To<FLU>(),
            orientation = fixedOrientation.To<FLU>()
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
            var renderOnlyFirst = false;
            var alreadyRendered = false;
            var doDraw = false;
            var alpha = 1.0f;
            int minN = 5;
            int halfLength = 1;

            // For every trajectory plan returned
            //int steps = 1; // For speedup execution on HoloLens
            //int steps = 30; // For execution on Editor 
            for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
            {
                if (poseIndex == response.arm_trajectory.trajectory.Length - 1)
                {
                    coroutineRunning = false;
                    //steps = 5;
                }
                renderOnlyFirst = false;
                alreadyRendered = false;
                if(response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length < minN)
                {
                    renderOnlyFirst = true;
                }
                else
                {
                    halfLength = (int)Mathf.Floor(response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length / 2.0f);
                }
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();

                    if(renderOnlyFirst && !alreadyRendered)
                    {
                        doDraw = true;
                        alreadyRendered = true;
                    }
                    else if(!renderOnlyFirst && jointConfigIndex % halfLength == 0)
                    {
                        doDraw = true;
                    }

                    lastJointState = result;

                    if(doDraw)
                    {
                        jointState.position = new double[9];
                        for (int k = 0; k < result.Length; k++)
                        {
                            jointState.position[k] = jointPositions[k];
                        }
                        jointState.position[7] = 0.0f;
                        jointState.position[8] = 0.0f;
                        drawing3 = Drawing3d.Create(20.0f, mat);
                        robotVisualization.DrawGhost(drawing3, jointState, new Color(0.7529f, 0.7529f, 0.7529f, alpha));
                        alpha -= 0.1f;

                        doDraw = false;
                    }
                }
            }
            yield return new WaitForSeconds(1.0f);
        }
        
    }

    public void Spawn(Vector3 spawnPosition)
    {
        robot.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0,-180.0f,0));
        robot.SetActive(true);
    }

}