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

    // Wooden Pieces
    public GameObject stoolSideLeft;
    public GameObject piece1;
    public GameObject piece2;

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
    public GameObject[] ghostPrefabs;

    // Offset variables for picking and placing objects
    private readonly Vector3 liftOffset = Vector3.up * 0.1f;
    private readonly float depthOffset = 0.135f;
    private readonly float heightOffset = 0.295f;

    // Other scene objects (buttons, marker)
    public GameObject imageTarget;
    public GameObject backButton;

    // Utility variables
    private GameObject[] tools;
    private GameObject[] components;
    private GameObject[] pieces;
    private Vector3[] componentsInitialPositions;
    private Quaternion[] componentsInitialRotations;
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
        controller.Init(robot, steps, urdfRobot, drawingMat, ghostPrefabs, renderMode);

        // Fill arrays with scene objects
        tools = new GameObject[1];
        tools[0] = screwdriver;

        components = new GameObject[3];
        components[0] = screwbox1;
        components[1] = screwbox2;
        components[2] = screwbox3;

        // Store initial values of components positions in order to reset positions
        int i = 0;
        componentsInitialPositions = new Vector3[components.Length];
        componentsInitialRotations = new Quaternion[components.Length];
        foreach(GameObject component in components)
        {
            componentsInitialPositions[i] = new Vector3(
                component.transform.localPosition.x, 
                component.transform.localPosition.y, 
                component.transform.localPosition.z);
            componentsInitialRotations[i] = new Quaternion(
                component.transform.localRotation.x,
                component.transform.localRotation.y,
                component.transform.localRotation.z,
                component.transform.localRotation.w
                );
            i++;
        }

        pieces = new GameObject[3];
        pieces[0] = piece1;
        pieces[1] = piece2;
        pieces[2] = stoolSideLeft;

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

            if(!components[ID].activeSelf)
            {
                components[ID].transform.localPosition = componentsInitialPositions[ID];
                components[ID].transform.localRotation = componentsInitialRotations[ID];
                // Reactivate tool and make it physically interactable again
                components[ID].GetComponent<Rigidbody>().isKinematic = false;
                components[ID].SetActive(true);
            }
            
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
        else // Default case
        {
            // Pick Pose
            pickPosition = placePositionMiddle.transform.localPosition + liftOffset;
            pickOrientation = Quaternion.Euler(-180, 0, 0);

            // Place Pose
            placePosition = pickPosition;
            placeOrientation = pickOrientation;
        }

        // Choose for which arm to plan based on position of object to pick
        if (pickPosition.x > 0)
        {
            arm = "right";
        }

        // Send motion planning request to ROS
        ActionServiceRequest request = null;
        request = controller.PlanningRequest(arm, op, pickPosition, placePosition, pickOrientation, placeOrientation);
        ros.SendServiceMessage<ActionServiceResponse>(request.arm + "_group/" + plannerServiceName, request, controller.ROSServiceResponse);

        if(renderMode == (int)BaxterController.RenderModes.FinalPose)
        {
            yield return new WaitForSeconds(2.0f);
            DeleteHologram(op);
        }

        // Wait for completion of trajectory execution
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

        if (renderMode != (int)BaxterController.RenderModes.FinalPose)
        {
            DeleteHologram(op);
        }
    }

    private void DeleteHologram(string op)
    {
        // Remove hologram of current object after action is done
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