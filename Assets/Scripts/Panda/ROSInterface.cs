using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.PandaUnity;

public class ROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject robot_hologram;
    public GameObject ground;
    public GameObject ground_holo;
    private List<Transform> pickPoses;
    private List<GameObject> instantiatedCubes;
    private Transform[] placePoses;

    // Canvas and interface objects
    private GameObject poseMarker;
    private GameObject[] buttons;

    // Variables required for ROS communication
    public string plannerServiceName = "motion_planner_service";

    private RobotController controller;
    private RobotController controller_hologram;

    // Utility variables
    private Vector3[] buttonsOffsets = { new Vector3(0.5f, 0.65f, 0.0f), new Vector3(0.5f, 0.5f, 0.0f) };
    private int nPick = 0;
    private int nPickMax = 4;
    private int nPlace = 1;
    private float deltaT = 3.0f;


    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Instantiate Robot Controller
        controller = gameObject.AddComponent<RobotController>();
        controller.Init(robot, ground.transform);

        // Instantiate Holographic Robot  Controller
        controller_hologram = gameObject.AddComponent<RobotController>();
        controller_hologram.Init(robot_hologram, ground_holo.transform);

        // Pick poses
        pickPoses = new List<Transform>();
        int i = 0;
        for (i = 0; i < nPick; i++)
        {
            pickPoses.Add(ground.transform.GetChild(i).gameObject.transform);
        }
        instantiatedCubes = new List<GameObject>();

        // Place poses
        placePoses = new Transform[nPlace];
        int k = 0;
        for (int j = i; j < i + nPlace; j++)
        {
            placePoses[k] = ground.transform.GetChild(j).gameObject.transform;
            k++;
        }

        // Get reference to Canvas layer
        var buttonsLayer = GameObject.Find("Canvas/Buttons");
        int numButtons = buttonsLayer.transform.childCount;
        buttons = new GameObject[numButtons];
        for (int j = 0; j < numButtons; j++)
        {
            var button = buttonsLayer.transform.GetChild(j).gameObject;
            buttons[j] = button;
        }

        // Get reference to PoseMarket object
        poseMarker = GameObject.Find("Scene/PoseMarker");

    }

    public void Spawn()
    {
        var markerPosition = poseMarker.transform.position;
        poseMarker.GetComponent<Behaviour>().enabled = false;
        controller.Spawn(markerPosition);
        controller_hologram.Spawn(markerPosition);
        poseMarker.SetActive(false);
        for(int i=0; i<buttons.Length; i++)
        {
            buttons[i].transform.SetPositionAndRotation(poseMarker.transform.position + buttonsOffsets[i], Quaternion.Euler(0, 0, 0));
            buttons[i].SetActive(true);
        }
    }

    public void AddCube()
    {
        if(nPick < nPickMax)
        {
            GameObject newCube = Instantiate(Resources.Load("Prefabs/Pick"+(nPick+1).ToString()), placePoses[0].position, Quaternion.Euler(0, 0, 0)) as GameObject;
         
            newCube.transform.parent = ground.transform;
            pickPoses.Add(newCube.transform);
            nPick++;
        }
    }

    public void Solve()
    {
        if(nPick >= 1)
        {
            // Spawn instantiated cube holograms
            for (int i = 1; i <= nPick; i++)
            {
                GameObject newCube = Instantiate(Resources.Load("Prefabs/Pick" + i.ToString()), pickPoses[i - 1].position, pickPoses[i - 1].rotation) as GameObject;
                var newMaterial = Resources.Load("Materials/Pick" + i.ToString() + "_holo") as Material;
                newCube.GetComponent<Renderer>().material = newMaterial;
                newCube.transform.parent = ground_holo.transform;
                newCube.layer = 11;

                instantiatedCubes.Add(newCube);
            }
            if (!controller.coroutineRunning)
            {
                StartCoroutine(PlanActions());
            }
        } 
    }

    private IEnumerator PlanActions()
    {
        PlannerServiceRequest request = null;
        for (int i=0; i<nPick; i++)
        {
            request = controller.PickAndPlaceService(
                i, 
                pickPoses[i].localPosition,
                placePoses[0].localPosition
            );
            //ros.SendServiceMessage<PlannerServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);
            ros.SendServiceMessage<PlannerServiceResponse>(plannerServiceName, request, plannerCallback);
            while (controller.coroutineRunning)
            {
                yield return new WaitForSeconds(0.1f);
            }
        }
        request = controller.GoHomeService();
        //ros.SendServiceMessage<PlannerServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);
        ros.SendServiceMessage<PlannerServiceResponse>(plannerServiceName, request, plannerCallback);

        // Destroy all instantiated cube holograms
        foreach(GameObject cube in instantiatedCubes) {
            Destroy(cube);
        }
        instantiatedCubes.Clear();
    }

    private void plannerCallback(PlannerServiceResponse response)
    {
        StartCoroutine(TrajectoryExecutioner(response));
    }

    private IEnumerator TrajectoryExecutioner(PlannerServiceResponse response)
    {
        StartCoroutine(controller_hologram.ExecuteTrajectories(response));
        yield return new WaitForSeconds(deltaT);
        StartCoroutine(controller.ExecuteTrajectories(response));
    }
}