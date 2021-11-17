using UnityEngine;

using System.Collections;

using RosMessageTypes.PandaUnityTest;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

public class ROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject ground;
    public UrdfRobot urdfRobot;

    public GameObject pick;
    public GameObject place;

    // Materials and prefabs
    public GameObject prefab;
    public Material drawingMat;

    // Canvas and interface objects
    public GameObject poseMarker;
    public GameObject addButton;
    public GameObject solveButton;

    // Variables required for ROS communication
    public string plannerServiceName = "motion_planner_service";

    private RobotController controller;

    // Utility variables
    private Vector3[] buttonsOffsets = { new Vector3(0.4f, 0.65f, 0.0f), new Vector3(0.4f, 0.5f, 0.0f) };

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<PlannerServiceRequest, PlannerServiceResponse>(plannerServiceName);

        // Instantiate Robot Controller
        controller = gameObject.AddComponent<RobotController>();
        controller.Init(robot, urdfRobot, ground.transform, drawingMat);
    }

    public void Spawn()
    {
        var markerPosition = poseMarker.transform.position;
        poseMarker.GetComponent<Behaviour>().enabled = false;
        controller.Spawn(markerPosition);
        poseMarker.SetActive(false);

        addButton.transform.SetPositionAndRotation(poseMarker.transform.position + buttonsOffsets[0], Quaternion.Euler(0, 0, 0));
        addButton.SetActive(true);

        solveButton.transform.SetPositionAndRotation(poseMarker.transform.position + buttonsOffsets[1], Quaternion.Euler(0, 0, 0));
        solveButton.SetActive(true);
    }

    public void Solve()
    {
        StartCoroutine(PlanActions());
    }

    private IEnumerator PlanActions()
    {
        PlannerServiceRequest request = null;
        request = controller.PickAndPlaceService(pick.transform.localPosition, place.transform.localPosition);
        
        ros.SendServiceMessage<PlannerServiceResponse>(plannerServiceName, request, plannerCallback);
        while (controller.coroutineRunning)
        {
            yield return new WaitForSeconds(0.1f);
        }
    }

    private void plannerCallback(PlannerServiceResponse response)
    {
        StartCoroutine(TrajectoryExecutioner(response));
    }

    private IEnumerator TrajectoryExecutioner(PlannerServiceResponse response)
    {
        StartCoroutine(controller.ExecuteTrajectories(response));
        yield return new WaitForSeconds(1.0f);
    }
}