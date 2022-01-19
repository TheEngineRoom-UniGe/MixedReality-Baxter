using System.Collections;

using RosMessageTypes.BaxterUnity;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;

public class AnticipatoryTrajectoryController : MonoBehaviour
{
    // Timing variables for rendering trajectories
    private float jointAssignmentWait = 0.005f;
    private float componentHandoverWait = 15.5f;
    private float pickPlaceWait = 5.5f;
    private float handoverWait = 6.0f;

    // Robot
    private GameObject baxter;
    private int numRobotJoints = 7;

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;
    private ArticulationBody[] leftHand;
    private ArticulationBody[] rightHand;
    private GameObject leftGripper;
    private GameObject rightGripper;

    // Hardcoded variables needed for referencing joints indices
    private double[] restPosition;
    private int[] leftIndices = { 4, 5, 2, 3, 6, 7, 8 };
    private int[] rightIndices = { 11, 12, 9, 10, 13, 14, 15 };

    // Utility variables
    public Queue leftCoroutineQueue;
    public Queue rightCoroutineQueue;
    private int steps;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
        Return
    };

    public void Init(GameObject baxter, int steps)
    {
        this.baxter = baxter;
        this.steps = steps;

        GetRobotReference();
        leftCoroutineQueue = new Queue();
        rightCoroutineQueue = new Queue();
    }

    private void GetRobotReference()
    {
        var side = "left";
        leftJointArticulationBodies = new ArticulationBody[numRobotJoints];
        string upper_shoulder = "base/" + side + "_arm_mount/" + side + "_upper_shoulder";
        leftJointArticulationBodies[0] = baxter.transform.Find(upper_shoulder).GetComponent<ArticulationBody>();

        string lower_shoudler = upper_shoulder + "/" + side + "_lower_shoulder";
        leftJointArticulationBodies[1] = baxter.transform.Find(lower_shoudler).GetComponent<ArticulationBody>();

        string upper_elbow = lower_shoudler + "/" + side + "_upper_elbow";
        leftJointArticulationBodies[2] = baxter.transform.Find(upper_elbow).GetComponent<ArticulationBody>();

        string lower_elbow = upper_elbow + "/" + side + "_lower_elbow";
        leftJointArticulationBodies[3] = baxter.transform.Find(lower_elbow).GetComponent<ArticulationBody>();

        string upper_forearm = lower_elbow + "/" + side + "_upper_forearm";
        leftJointArticulationBodies[4] = baxter.transform.Find(upper_forearm).GetComponent<ArticulationBody>();

        string lower_forearm = upper_forearm + "/" + side + "_lower_forearm";
        leftJointArticulationBodies[5] = baxter.transform.Find(lower_forearm).GetComponent<ArticulationBody>();

        string wrist = lower_forearm + "/" + side + "_wrist";
        leftJointArticulationBodies[6] = baxter.transform.Find(wrist).GetComponent<ArticulationBody>();

        string hand = wrist + "/" + side + "_hand";
        string gripper_base = hand + "/" + side + "_gripper_base";
        leftGripper = baxter.transform.Find(gripper_base + "/" + side + "_gripper").gameObject;
        // Find left and right fingers
        string right_gripper_finger = gripper_base + "/l_gripper_r_finger";
        string left_gripper_finger =  gripper_base + "/l_gripper_l_finger";

        leftHand = new ArticulationBody[2];
        leftHand[0] = baxter.transform.Find(left_gripper_finger).GetComponent<ArticulationBody>();
        leftHand[1] = baxter.transform.Find(right_gripper_finger).GetComponent<ArticulationBody>();

        side = "right";
        rightJointArticulationBodies = new ArticulationBody[numRobotJoints];
        upper_shoulder = "base/" + side + "_arm_mount/" + side + "_upper_shoulder";
        rightJointArticulationBodies[0] = baxter.transform.Find(upper_shoulder).GetComponent<ArticulationBody>();

        lower_shoudler = upper_shoulder + "/" + side + "_lower_shoulder";
        rightJointArticulationBodies[1] = baxter.transform.Find(lower_shoudler).GetComponent<ArticulationBody>();

        upper_elbow = lower_shoudler + "/" + side + "_upper_elbow";
        rightJointArticulationBodies[2] = baxter.transform.Find(upper_elbow).GetComponent<ArticulationBody>();

        lower_elbow = upper_elbow + "/" + side + "_lower_elbow";
        rightJointArticulationBodies[3] = baxter.transform.Find(lower_elbow).GetComponent<ArticulationBody>();

        upper_forearm = lower_elbow + "/" + side + "_upper_forearm";
        rightJointArticulationBodies[4] = baxter.transform.Find(upper_forearm).GetComponent<ArticulationBody>();

        lower_forearm = upper_forearm + "/" + side + "_lower_forearm";
        rightJointArticulationBodies[5] = baxter.transform.Find(lower_forearm).GetComponent<ArticulationBody>();

        wrist = lower_forearm + "/" + side + "_wrist";
        rightJointArticulationBodies[6] = baxter.transform.Find(wrist).GetComponent<ArticulationBody>();

        hand = wrist + "/" + side + "_hand";
        gripper_base = hand + "/" + side + "_gripper_base";
        rightGripper = baxter.transform.Find(gripper_base + "/" + side + "_gripper").gameObject;
        // Find left and right fingers
        right_gripper_finger = gripper_base + "/r_gripper_r_finger";
        left_gripper_finger =  gripper_base + "/r_gripper_l_finger";

        rightHand = new ArticulationBody[2];
        rightHand[0] = baxter.transform.Find(left_gripper_finger).GetComponent<ArticulationBody>();
        rightHand[1] = baxter.transform.Find(right_gripper_finger).GetComponent<ArticulationBody>();
    }

    public void SpawnRobotAndInterface(Vector3 spawnPosition)
    {
        baxter.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0, -180.0f, 0));
        baxter.SetActive(true);
    }

    private void CloseGripper(string hand)
    {
        var gripper = leftHand;
        if(hand == "right")
        {
            gripper = rightHand;
        }
        
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = -0.005f;
        rightDrive.target = 0.005f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    private void OpenGripper(string hand)
    {
        var gripper = leftHand;
        if (hand == "right")
        {
            gripper = rightHand;
        }

        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.025f;
        rightDrive.target = -0.025f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    public void JointStateServiceResponse(JointStateServiceResponse response)
    {
        var msg = response.joint_state_msg;
        restPosition = new double[msg.position.Length];
        {
            for(int i = 0; i<msg.position.Length; i++)
            {
                restPosition[i] = msg.position[i];
            }
        }
        GoToRestPosition("both");
    }

    ArmJointsMsg InitialJointConfig(string arm)
    {
        ArmJointsMsg joints = new ArmJointsMsg();
        var indices = (arm == "left") ? leftIndices : rightIndices;

        joints.angles = new double[numRobotJoints];
        for(int i = 0; i < numRobotJoints; i++)
        {
            joints.angles[i] = Mathf.Rad2Deg * (float)restPosition[indices[i]];
        }
        return joints;
    }

    public void GoToRestPosition(string whichArm)
    {
        if (whichArm == "left")
        {
            StartCoroutine(GoToRest(leftIndices, whichArm));
        }
        else if (whichArm == "right")
        {
            StartCoroutine(GoToRest(rightIndices, whichArm));
        }
        else
        {
            StartCoroutine(GoToRest(leftIndices, "left"));
            StartCoroutine(GoToRest(rightIndices, "right"));
        }
    }

    private IEnumerator GoToRest(int[] indices, string arm)
    {
        float[] target = new float[indices.Length];
        for(int i = 0; i<indices.Length; i++)
        {
            target[i] = Mathf.Rad2Deg * (float)restPosition[indices[i]];
        }
        float[] lastJointState = {0,0,0,0,0,0,0}; 
        var steps = 100;
        var jointArticulationBodies = leftJointArticulationBodies;
        if(arm == "right")
        {
            jointArticulationBodies = rightJointArticulationBodies;
        }
        for (int i = 0; i <= steps; i++)
        {
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * (float)(1.0f / steps) * (float)i;
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWait);
        }
        OpenGripper(arm);
    }

    // Routine to generate motion planning request
    public ActionServiceRequest PlanningRequest(string arm, string op, Vector3 pickPos, Vector3 placePos, Quaternion pickOr, Quaternion placeOr)
    {
        ActionServiceRequest request = new ActionServiceRequest();
        request.action = op;
        request.arm = arm;

        request.pick_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = pickPos.To<FLU>(),
            orientation = pickOr.To<FLU>()
        };

        request.place_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = placePos.To<FLU>(),
            orientation = placeOr.To<FLU>()
        };

        request.joints = InitialJointConfig(arm);

        return request;
    }
 
    public void ROSServiceResponse(ActionServiceResponse response)
    {
        if (response.arm_trajectory.trajectory.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteAnticipatoryTrajectory(response));
        }
        else
        {
            Debug.Log("No trajectory returned from MoveIt.");
        }
    }

    private IEnumerator ExecuteAnticipatoryTrajectory(ActionServiceResponse response)
    {
        var arm = response.arm_trajectory.arm;
        var initialJointConfig = InitialJointConfig(arm);
        var lastJointState = initialJointConfig.angles;

        // For every trajectory plan returned
        var jointArticulationBodies = leftJointArticulationBodies;
        if (arm == "right")
        {
            jointArticulationBodies = rightJointArticulationBodies;
        }
        for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
        {
            // For every robot pose in trajectory plan
            for (int jointConfigIndex = 0; jointConfigIndex < response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositionsRad = response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                var jointPositionsDeg = new double[jointPositionsRad.Length];
                for (int i = 0; i < lastJointState.Length; i++)
                {
                    jointPositionsDeg[i] = jointPositionsRad[i] * Mathf.Rad2Deg;
                }
                for (int i = 0; i <= steps; i++)
                {
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = (float)(lastJointState[joint] + (jointPositionsDeg[joint] - lastJointState[joint]) * (1.0f / steps) * i);
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    yield return new WaitForSeconds(jointAssignmentWait);
                }
                lastJointState = jointPositionsDeg;
            }
            // Make sure gripper is open at the beginning
            if (poseIndex == (int)Poses.PreGrasp || poseIndex == (int)Poses.Place)
            {
                yield return new WaitForSeconds(0.5f);
                OpenGripper(arm);
            }
            // Close gripper on object grasping
            if (poseIndex == (int)Poses.Grasp)
            {
                yield return new WaitForSeconds(0.5f);
                CloseGripper(arm);
            }
            // Handle different cases based on the executed action
            if (response.action == "pick_and_place" && poseIndex == (int)Poses.Place)
            {
                yield return new WaitForSeconds(pickPlaceWait);
                EndTrajectoryExecution(arm);
            }
            else if (response.action == "tool_handover" && poseIndex == (int)Poses.Move)
            {
                yield return new WaitForSeconds(handoverWait);
                EndTrajectoryExecution(arm);
            }
            else if (response.action == "component_handover")
            {
                if (poseIndex == (int)Poses.Move)
                {
                    yield return new WaitForSeconds(componentHandoverWait);
                }
                else if (poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(0.5f);
                    EndTrajectoryExecution(arm);
                }
            }
            else if (response.action == "put_back" && poseIndex == (int)Poses.Place)
            {
                yield return new WaitForSeconds(pickPlaceWait);
                EndTrajectoryExecution(arm);
            }
        }
    }

    private void EndTrajectoryExecution(string arm)
    {
        if (arm == "left")
        {
            leftCoroutineQueue.Dequeue();
        }
        else
            rightCoroutineQueue.Dequeue();

        OpenGripper(arm);
        
    }
}