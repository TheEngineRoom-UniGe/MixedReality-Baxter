using System.Collections;
using System.Linq;
using System.Collections.Generic;

using RosMessageTypes.BaxterUnity;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using JointState = RosMessageTypes.Sensor.JointStateMsg;

using Unity.Robotics.Visualizations;
using Unity.Robotics.UrdfImporter;

using UnityEngine;

public class BaxterController : MonoBehaviour
{
    // Timing variables for rendering trajectories
    private float jointAssignmentWait = 0.005f;
    private float placeWait = 12.0f;
    private float drawTime = 12.0f;

    // Robot
    private GameObject baxter;
    private UrdfRobot urdfRobot;
    private int numRobotJoints = 7;
    private int renderMode;

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
    private string[] left_joint_names = {
            "left_s0",
            "left_s1",
            "left_e0",
            "left_e1",
            "left_w0",
            "left_w1",
            "left_w2",
            "l_gripper_r_finger_joint",
            "l_gripper_l_finger_joint",
    };
    private string[] right_joint_names = {
            "right_s0",
            "right_s1",
            "right_e0",
            "right_e1",
            "right_w0",
            "right_w1",
            "right_w2",
            "r_gripper_r_finger_joint",
            "r_gripper_l_finger_joint",
    };

    // Utility variables
    public bool leftCoroutineRunning = false;
    public bool rightCoroutineRunning = false;
    private int steps;

    // Drawing components
    private RobotVisualization robotVisualization;
    private Drawing3d drawing3;
    private Material mat;
    private int minN = 6;
    private float red = 0.5471f;
    private GameObject[] ghostPrefabs;

    public enum RenderModes
    {
        GhostTrajectory,
        FinalPose,
        AnticipatoryTrajectory
    };

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
        Return
    };

    public void Init(GameObject baxter, int steps, UrdfRobot urdfRobot, Material mat, GameObject[] ghostPrefabs, int renderMode)
    {
        this.baxter = baxter;
        this.steps = steps;
        this.urdfRobot = urdfRobot;
        this.mat = mat;
        this.renderMode = renderMode;
        this.ghostPrefabs = ghostPrefabs;

        GetRobotReference();

        robotVisualization = new RobotVisualization(this.urdfRobot);

        drawing3 = Drawing3d.Create(10.0f, mat);
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

        if (arm == "left")
        {
            leftCoroutineRunning = true;
        }
        else
            rightCoroutineRunning = true;

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
            if(renderMode == (int)RenderModes.GhostTrajectory)
            {
                StartCoroutine(RenderGhostTrajectory(response));
            }
            else if(renderMode == (int)RenderModes.FinalPose)
            {
                StartCoroutine(RenderFinalPoseOnly(response));
            }
            else if(renderMode == (int)RenderModes.AnticipatoryTrajectory)
            {
                StartCoroutine(ExecuteAnticipatoryTrajectory(response));
            } 
        }
        else
        {
            Debug.Log("No trajectory returned from MoveIt.");
        }
    }

    private IEnumerator RenderGhostTrajectory(ActionServiceResponse response)
    {
        var arm = response.arm_trajectory.arm;
        var initialJointConfig = InitialJointConfig(arm);
        double[] lastJointState = initialJointConfig.angles;
        var instantiatedGhosts = new List<GameObject>();
        var gripper = leftGripper;
        var finalPose = (int)Poses.Return;
        var drawPose = (int)Poses.Place;

        GameObject ghostPrefab = null;
        if(response.action == "pick_and_place")
        {
            if(response.seq < 2)
            {
                ghostPrefab = ghostPrefabs[0];
            }
            else
            {
                ghostPrefab = ghostPrefabs[1];
            }
        }

        else if(response.action == "component_handover")
        {
            ghostPrefab = ghostPrefabs[2];

        }
        else if(response.action == "tool_handover")
        {
            ghostPrefab = ghostPrefabs[3];
            finalPose = (int)Poses.Place;
            drawPose = (int)Poses.Move;
        }
        
        var doDraw = false;
        var sign = 1;

        // For every trajectory plan returned
        var jointState = new JointState();
        jointState.name = left_joint_names;
        var jointArticulationBodies = leftJointArticulationBodies;
        if (arm == "right")
        {
            sign = -1;
            gripper = rightGripper;
            jointState.name = right_joint_names;
            jointArticulationBodies = rightJointArticulationBodies;
        }
        
        for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
        {
            // For every robot pose in trajectory plan
            for (int jointConfigIndex = 0; jointConfigIndex < response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositions = response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();

                // If trajectory is short, draw one every 3 poses
                if (response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length < minN && jointConfigIndex % 3 == 0)
                {
                    doDraw = true;
                }
                // If longer, draw one every 5 poses
                else if (response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length >= minN && jointConfigIndex % 5 == 0)
                {
                    doDraw = true;
                }

                // If place pose, draw last position
                if (poseIndex == drawPose && jointConfigIndex == response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length - 1)
                {
                    doDraw = true;
                }

                for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive = jointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = (float) result[joint];
                    jointArticulationBodies[joint].xDrive = joint1XDrive;
                }

                yield return new WaitForSeconds(jointAssignmentWait * 5.0f);

                lastJointState = result;

                if (doDraw)
                {
                    jointState.position = new double[9];
                    for (int k = 0; k < result.Length; k++)
                    {
                        jointState.position[k] = jointPositions[k];
                        if (k == 0)
                        {
                            jointState.position[k] += sign * Mathf.PI / 4.0f;
                        }
                       
                    }
                    if (poseIndex > (int)Poses.Grasp && poseIndex < finalPose)
                    {
                        var newGo = Instantiate(ghostPrefab, gripper.transform.position, gripper.transform.rotation);
                        instantiatedGhosts.Add(newGo);
                    }

                    drawing3 = Drawing3d.Create(drawTime, mat);
                    robotVisualization.DrawGhost(drawing3, jointState, new Color(red, 0, 0, 1.0f));

                    doDraw = false;
                }
                
            }
        }
        
        yield return new WaitForSeconds(drawTime);
        foreach(GameObject go in instantiatedGhosts)
        {
            Destroy(go);
        }
        instantiatedGhosts.Clear();
        EndTrajectoryExecution(arm);
    }

    private IEnumerator RenderFinalPoseOnly(ActionServiceResponse response)
    {
        var arm = response.arm_trajectory.arm;
        var initialJointConfig = InitialJointConfig(arm);
        double[] lastJointState = initialJointConfig.angles;
        var instantiatedGhosts = new List<GameObject>();
        var sign = 1;
        var gripper = leftGripper;
        var finalPose = (int)Poses.Place;

        GameObject ghostPrefab = null;
        if (response.action == "pick_and_place")
        {
            if (response.seq < 2)
            {
                ghostPrefab = ghostPrefabs[0];
            }
            else
            {
                ghostPrefab = ghostPrefabs[1];
            }
        }

        else if (response.action == "component_handover")
        {
            ghostPrefab = ghostPrefabs[2];
            finalPose = (int)Poses.Move;

        }
        else if (response.action == "tool_handover")
        {
            ghostPrefab = ghostPrefabs[3];
            finalPose = (int)Poses.Move;
        }

        // For every trajectory plan returned
        var jointState = new JointState();
        jointState.name = left_joint_names;
        var jointArticulationBodies = leftJointArticulationBodies;
        if (arm == "right")
        {
            sign = -1;
            gripper = rightGripper;
            jointState.name = right_joint_names;
            jointArticulationBodies = rightJointArticulationBodies;
        }

        for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
        {
            // For every robot pose in trajectory plan
            for (int jointConfigIndex = 0; jointConfigIndex < response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositions = response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();

                for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive = jointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = (float)result[joint];
                    jointArticulationBodies[joint].xDrive = joint1XDrive;
                }

                yield return new WaitForSeconds(jointAssignmentWait * 5.0f);

                // Render only final pose of the action
                if (poseIndex == finalPose && jointConfigIndex == response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length - 1)
                {
                    jointState.position = new double[9];
                    for (int k = 0; k < result.Length; k++)
                    {
                        jointState.position[k] = jointPositions[k];
                        if (k == 0)
                        {
                            jointState.position[k] += sign * Mathf.PI / 4.0f;
                        }

                    }
                                        
                    drawing3 = Drawing3d.Create(drawTime, mat);
                    robotVisualization.DrawGhost(drawing3, jointState, new Color(red, 0, 0, 1.0f));

                    var newGo = Instantiate(ghostPrefab, gripper.transform.position, gripper.transform.rotation);
                    instantiatedGhosts.Add(newGo);
                }
            }
        }

        yield return new WaitForSeconds(drawTime - 0.5f);
        foreach (GameObject go in instantiatedGhosts)
        {
            Destroy(go);
        }
        instantiatedGhosts.Clear();
        EndTrajectoryExecution(arm);
    }

    private IEnumerator ExecuteAnticipatoryTrajectory(ActionServiceResponse response)
    {
        var arm = response.arm_trajectory.arm;
        var initialJointConfig = InitialJointConfig(arm);
        var steps = this.steps;
        double[] lastJointState = initialJointConfig.angles;

        // For every trajectory plan returned
        var jointArticulationBodies = leftJointArticulationBodies;
        if (arm == "right")
        {
            jointArticulationBodies = rightJointArticulationBodies;
        }
        yield return new WaitForSeconds(0.1f);
        for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
        {
            // Slow down rendering when returning to home position
            if(poseIndex == response.arm_trajectory.trajectory.Length - 1)
            {
                steps *= 2;
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
                lastJointState = result;

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
                yield return new WaitForSeconds(placeWait);
                EndTrajectoryExecution(arm);
            }
            else if (response.action == "tool_handover" && poseIndex == (int)Poses.Move)
            {
                yield return new WaitForSeconds(placeWait);
                EndTrajectoryExecution(arm);

            }
            else if (response.action == "component_handover")
            {
                if (poseIndex == (int)Poses.Move)
                {
                    yield return new WaitForSeconds(placeWait);
                }
                else if (poseIndex == (int)Poses.Return)
                {
                    EndTrajectoryExecution(arm);
                }
            }
            else if (response.action == "put_back" && poseIndex == (int)Poses.Place)
            {
                yield return new WaitForSeconds(placeWait);
                EndTrajectoryExecution(arm);
            }
        }
    }

    private void EndTrajectoryExecution(string arm)
    {
        if (arm == "left")
        {
            leftCoroutineRunning = false;
        }
        else
            rightCoroutineRunning = false;

        OpenGripper(arm);
        
    }
}