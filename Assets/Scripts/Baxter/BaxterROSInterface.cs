using System.Collections;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BaxterUnityTest;
using Unity.Robotics.UrdfImporter;

using Bool = RosMessageTypes.Std.BoolMsg;

public class BaxterROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject ground;
    
    // Variables for holo rendering
    public int renderMode;
    public int steps = 15;

    // Variables required for ROS communication
    public string plannerServiceName = "baxter_unity_motion_planner";
    public string jointStateServiceName = "baxter_joint_states";
    public string nextActionTopicName = "next_action";
    public string actionDoneTopicName = "action_done";

    // Tools
    public GameObject screwdriver;

    // Pieces
    public GameObject stoolSideLeft;

    // Components
    public GameObject screwbox1;
    public GameObject screwbox2;
    public GameObject screwbox3;

    // Place/handover positions
    public GameObject handoverPositionLeft;
    public GameObject handoverPositionRight;
    public GameObject placePositionMiddle;

    // Materials and URDF
    public UrdfRobot urdfRobot;
    public Material drawingMat;
    public GameObject ghostPrefab;

    // Offset variables for picking and placing objects
    private readonly Vector3 liftOffset = Vector3.up * 0.1f;
    private readonly Vector3 dropOffset = Vector3.up * 0.02f;
    private readonly float depthOffset = 0.135f;
    private readonly float heightOffset = 0.295f;

    // Other scene objects (buttons, marker)
    public GameObject imageTarget;
    public GameObject backButton;

    // Utility variables
    private GameObject[] tools;
    private GameObject[] components;
    private GameObject[] pieces;
    /*private Vector3[] pickInitialPositions;
    private Quaternion[] pickInitialRotations;
    private Vector3[] toolsInitialPositions;
    private Quaternion[] toolsInitialRotations;*/
    private Queue piecesIDQueue;
    private Queue componentsIDQueue;
    private Queue toolsIDQueue;

    private BaxterController controller;

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        
        ros.RegisterRosService<JointStateServiceRequest, JointStateServiceResponse>(jointStateServiceName);

        // Register to motion planner service
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>("left_group/" + plannerServiceName);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>("right_group/" + plannerServiceName);

        // Register to topics related to plan management
        ros.Subscribe<NextActionMsg>(nextActionTopicName, PlanNextAction);
        ros.RegisterPublisher<Bool>(actionDoneTopicName);

        // Instantiate Baxter Controller
        controller = gameObject.AddComponent<BaxterController>();
        controller.Init(robot, steps, urdfRobot, drawingMat, ghostPrefab, renderMode);

        // Fill arrays with scene objects
        tools = new GameObject[1];
        tools[0] = screwdriver;

        components = new GameObject[3];
        components[0] = screwbox1;
        components[1] = screwbox2;
        components[2] = screwbox3;

        pieces = new GameObject[1];
        pieces[0] = stoolSideLeft;

        // Request initial joint position from real robot controller
        var request = new JointStateServiceRequest();
        ros.SendServiceMessage<JointStateServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);

        //Instantiate queues of ids for correct rendering of objects
        toolsIDQueue = new Queue();
        componentsIDQueue = new Queue();
        piecesIDQueue = new Queue();
        
    }

    public void Spawn()
    {
        var imTargetPosition = imageTarget.transform.position;
        //imageTarget.GetComponent<ImageTargetBehaviour>().enabled = false;
        imageTarget.GetComponent<Behaviour>().enabled = false;
        imageTarget.SetActive(false);

        backButton.transform.SetPositionAndRotation(imTargetPosition + Vector3.left * 0.75f + Vector3.back * 0.3f, Quaternion.identity);
        backButton.SetActive(true);

        var spawnPosition = imTargetPosition + Vector3.forward * depthOffset - Vector3.up * heightOffset;
        controller.SpawnRobotAndInterface(spawnPosition);

        // Signal initialization complete
        ros.Publish(actionDoneTopicName, new Bool());
    }

    public void PlanNextAction(NextActionMsg msg)
    {
        StartCoroutine(PlanActionRoutine(msg));
    }

    private IEnumerator PlanActionRoutine(NextActionMsg msg)
    {
        // Extract planning parameters from message
        int ID = msg.id;
        string op = msg.op;

        Vector3 pickPosition;
        Vector3 placePosition;
        Quaternion pickOrientation;
        Quaternion placeOrientation;

        string arm = "left";

        if (op == "pick_and_place")
        {
            piecesIDQueue.Enqueue(ID);

            // Pick Pose
            pickPosition = pieces[ID].transform.localPosition + liftOffset;
            pickOrientation = Quaternion.Euler(180, 0, 0);

            // Place Pose
            placePosition = placePositionMiddle.transform.localPosition + liftOffset;
            placeOrientation = pickOrientation;
            /*if (ID == 0)
            {
                placeOrientation = Quaternion.Euler(180, -90, 0);
            }
            else if (ID == 1 || ID == 2)
            {
                placeOrientation = Quaternion.Euler(180, 90, 0);
            }*/
        }
        else if (op == "tool_handover")
        {
            toolsIDQueue.Enqueue(ID);

            // Pick Pose
            pickPosition = tools[ID].transform.localPosition + liftOffset;
            pickOrientation = Quaternion.Euler(-180.0f, 0.0f, 0.0f);

            var handoverPosition = handoverPositionLeft;
            if (pickPosition.x > 0)
            {
                handoverPosition = handoverPositionRight;
            }

            // Handover Pose
            placePosition = handoverPosition.transform.localPosition;
            placeOrientation = Quaternion.Euler(-90.0f, 90.0f, 90.0f);
        }
        else if (op == "component_handover")
        {
            componentsIDQueue.Enqueue(ID);

            /*pickPoses[ID].transform.localPosition = pickInitialPositions[ID];
            pickPoses[ID].transform.localRotation = pickInitialRotations[ID];*/
            // Reactivate tool and make it physically interactable again
            /*pickPoses[ID].GetComponent<Rigidbody>().isKinematic = false;
            pickPoses[ID].SetActive(true);*/

            // Pick Pose
            pickPosition = components[ID].transform.localPosition + liftOffset;
            pickOrientation = Quaternion.Euler(180, 90, 0);

            var handoverPosition = handoverPositionLeft;
            if (pickPosition.x > 0)
            {
                handoverPosition = handoverPositionRight;
            }

            // Handover Pose
            placePosition = handoverPosition.transform.localPosition;
            placeOrientation = Quaternion.Euler(180, 90, 0);
            
        }
        else // Put back case
        {
            /*toolsIDQueue.Enqueue(ID);

            var putBackPickPose = placePoses[4];
            if (toolsInitialPositions[ID].x > 0)
            {
                putBackPickPose = placePoses[5];
                arm = "right";
            }

            tools[ID].transform.localPosition = putBackPickPose.transform.localPosition;
            tools[ID].transform.localRotation = toolsInitialRotations[ID];
            // Reactivate tool and make it physically interactable again
            tools[ID].GetComponent<Rigidbody>().isKinematic = false;
            tools[ID].SetActive(true);*/
            
            // Pick Pose
            pickPosition = placePositionMiddle.transform.localPosition + liftOffset;
            pickOrientation = Quaternion.Euler(-180, 0, 0);

            // Place Pose
            //placePosition = toolsInitialPositions[ID] + liftOffset + toolsGraspOffsets[ID] + dropOffset;
            placePosition = pickPosition;
            placeOrientation = pickOrientation;
        }

        if (pickPosition.x > 0)
        {
            arm = "right";
        }

        ActionServiceRequest request = null;
        request = controller.PlanningRequest(arm, op, pickPosition, placePosition, pickOrientation, placeOrientation);
        ros.SendServiceMessage<ActionServiceResponse>(request.arm + "_group/" + plannerServiceName, request, controller.ROSServiceResponse);

        if(arm == "left")
        {
            while (controller.leftCoroutineRunning)
            {
                yield return new WaitForSeconds(0.1f);
            }
            ros.Publish(actionDoneTopicName, new Bool());
        }
        else
        {
            while (controller.rightCoroutineRunning)
            {
                yield return new WaitForSeconds(0.1f);
            }
            ros.Publish(actionDoneTopicName, new Bool());
        }
        
        if (op == "pick_and_place")
        {
            var currentPickID = (int)piecesIDQueue.Dequeue();
            pieces[currentPickID].GetComponent<Rigidbody>().isKinematic = true;
            pieces[currentPickID].SetActive(false);
        }
        else if (op == "component_handover")
        {
            var currentComponentID = (int)componentsIDQueue.Dequeue();
            components[currentComponentID].GetComponent<Rigidbody>().isKinematic = true;
            components[currentComponentID].SetActive(false);
        }
        else
        {
            var currentToolID = (int)toolsIDQueue.Dequeue();
            tools[currentToolID].GetComponent<Rigidbody>().isKinematic = true;
            tools[currentToolID].SetActive(false);
        }
    }

    public void ResetScene()
    {
        // Revert objects to initial positions and orientations
        /*for (int i = 0; i < pickPoses.Length; i++)
        {
            pickPoses[i].transform.localPosition = pickInitialPositions[i];
            pickPoses[i].transform.localRotation = pickInitialRotations[i];
            // Reactivate object and make it physically interactable again
            pickPoses[i].GetComponent<Rigidbody>().isKinematic = false;
            pickPoses[i].SetActive(true);
        }

        // Revert tools to initial positions and orientations
        for (int i = 0; i < tools.Length; i++)
        {
            tools[i].transform.localPosition = toolsInitialPositions[i];
            tools[i].transform.localRotation = toolsInitialRotations[i];
            // Reactivate tool and make it physically interactable again
            tools[i].GetComponent<Rigidbody>().isKinematic = false;
            tools[i].SetActive(true);
        }

        var request = new JointStateServiceRequest();
        ros.SendServiceMessage<JointStateServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);*/
    }
}