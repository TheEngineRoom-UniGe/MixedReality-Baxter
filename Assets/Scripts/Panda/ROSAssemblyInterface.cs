using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.PandaUnity;
using Microsoft.MixedReality.Toolkit.UI;

public class ROSAssemblyInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject robot_hologram;

    public GameObject ground;
    public GameObject ground_holo;

    public GameObject chair;

    private GameObject piece1;
    private GameObject piece2;
    private GameObject assembled;

    //private List<Transform> pickPoses;
    private List<GameObject> instantiatedCubes;
    //private Transform[] placePoses;

    // Canvas and interface objects
    public GameObject poseMarker;
    private GameObject[] buttons;

    // Variables required for ROS communication
    public string plannerServiceName = "motion_planner_service";

    private RobotController controller;
    private RobotController controller_hologram;

    // Utility variables
    private Vector3[] buttonsOffsets = { new Vector3(0.5f, 0.65f, 0.0f), new Vector3(0.5f, 0.5f, 0.0f), new Vector3(0.65f, 0.65f, 0.0f) };

    private float deltaT = 3.0f;

    private int assemblyStep = 0;
    private int assemblyCount = 0;
    private int stepSolvedCount = 0;
    private bool prevStepSolved = true;
    private Vector3 piece1InitPosition;
    private Vector3 piece2InitPosition;
    private Quaternion piece1InitRot;
    private Quaternion piece2InitRot;


    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Instantiate Robot Controller
        controller = gameObject.AddComponent<RobotController>();
        controller.Init(robot, ground.transform);

        // Instantiate Holographic Robot  Controller
        //controller_hologram = gameObject.AddComponent<RobotController>();
        //controller_hologram.Init(robot_hologram, ground_holo.transform);

        // Get reference to Canvas layer
        var buttonsLayer = GameObject.Find("Canvas/Buttons");
        int numButtons = buttonsLayer.transform.childCount;
        buttons = new GameObject[numButtons];
        for (int j = 0; j < numButtons; j++)
        {
            var button = buttonsLayer.transform.GetChild(j).gameObject;
            buttons[j] = button;
        }
    }

    public void Spawn()
    {
        var markerPosition = poseMarker.transform.position;
        poseMarker.GetComponent<Behaviour>().enabled = false;
        controller.Spawn(markerPosition);
        //controller_hologram.Spawn(markerPosition);
        poseMarker.SetActive(false);
        for(int i=0; i<buttons.Length; i++)
        {
            buttons[i].transform.SetPositionAndRotation(poseMarker.transform.position + buttonsOffsets[i], Quaternion.Euler(0, 0, 0));
            buttons[i].SetActive(true);
        }
    }

    public void NextAssemblyStep()
    {
        if(assemblyCount < chair.transform.childCount - 1 && prevStepSolved)
        {
            assemblyStep += 1;
            if (assemblyStep > 1)
            {
                piece1.SetActive(false);
                piece2.SetActive(false);
                assembled.SetActive(false);
            }

            prevStepSolved = false;

            piece1 = chair.transform.GetChild(assemblyCount).gameObject;
            piece2 = chair.transform.GetChild(assemblyCount + 1).gameObject;
            assembled = chair.transform.GetChild(assemblyCount + 2).gameObject;

            piece1InitPosition = new Vector3(piece1.transform.position.x, piece1.transform.position.y, piece1.transform.position.z);
            piece2InitPosition = new Vector3(piece2.transform.position.x, piece2.transform.position.y, piece2.transform.position.z);
            piece1InitRot = new Quaternion(
                piece1.transform.rotation.x,
                piece1.transform.rotation.y,
                piece1.transform.rotation.z,
                piece1.transform.rotation.w
            );
            piece2InitRot = new Quaternion(
                piece2.transform.rotation.x,
                piece2.transform.rotation.y,
                piece2.transform.rotation.z,
                piece2.transform.rotation.w
            );

            piece1.SetActive(true);
            piece2.SetActive(true);
            assembled.SetActive(true);

            assemblyCount += 3;
        }
        else if(assemblyCount == chair.transform.childCount - 1)
        {
            piece1.SetActive(false);
            piece2.SetActive(false);
            assembled.SetActive(false);
            assembled = chair.transform.GetChild(chair.transform.childCount - 1).gameObject;
            assembled.SetActive(true);
            foreach(GameObject button in buttons)
            {
                button.SetActive(false);
            }
        }
    }

    public void ResetStep()
    {
        piece1.transform.SetPositionAndRotation(piece1InitPosition, piece1InitRot);
        piece1.GetComponent<ObjectManipulator>().enabled = true;

        piece2.transform.SetPositionAndRotation(piece2InitPosition, piece2InitRot);
        piece2.GetComponent<ObjectManipulator>().enabled = true;

        stepSolvedCount = 0;
    }

    public void CheckInPosition(int id)
    {
        if (id == 1)
        {
            var childPose = assembled.transform.GetChild(2 * (assemblyStep - 1)).transform;

            var rotA = new Vector4(
                piece1.transform.localRotation.x,
                piece1.transform.localRotation.y,
                piece1.transform.localRotation.z,
                piece1.transform.localRotation.w
            );
            var rotB = new Vector4(
                childPose.localRotation.x,
                childPose.localRotation.y,
                childPose.localRotation.z,
                childPose.localRotation.w
            );

            var positionDiff = (piece1.transform.position - childPose.position).magnitude;
            var rotDiff = (rotA - rotB).magnitude;

            if(positionDiff < 0.05f && rotDiff < 10e-1)
            {
                piece1.transform.SetPositionAndRotation(childPose.position, childPose.localRotation);
                piece1.GetComponent<ObjectManipulator>().enabled = false;
                
                stepSolvedCount += 1;
            }
        }
        else
        {
            var childPose = assembled.transform.GetChild(2 * (assemblyStep - 1) + 1).transform;

            var rotA = new Vector4(
                piece2.transform.localRotation.x,
                piece2.transform.localRotation.y,
                piece2.transform.localRotation.z,
                piece2.transform.localRotation.w
            );
            var rotB = new Vector4(
                childPose.localRotation.x,
                childPose.localRotation.y,
                childPose.localRotation.z,
                childPose.localRotation.w
            );

            var positionDiff = (piece2.transform.position - childPose.position).magnitude;
            var rotDiff = (rotA - rotB).magnitude;

            if (positionDiff < 0.05f && rotDiff < 10e-1)
            {
                piece2.transform.SetPositionAndRotation(childPose.position, childPose.localRotation);
                piece2.GetComponent<ObjectManipulator>().enabled = false;

                stepSolvedCount += 1;
            }
        }

        if(stepSolvedCount == 2)
        {
            stepSolvedCount = 0;
            prevStepSolved = true;
        }
    }

    public void SolveStep()
    {
        if(stepSolvedCount < 1)
        {
            StartCoroutine(PlanAction());
        }
        stepSolvedCount += 1;
    }

    private IEnumerator PlanAction()
    {
        int rnd = Random.Range(1, 3);
        Transform childPose;
        Vector3 pickPosition;
        Vector3 placePosition;
        if(rnd == 1)
        {
            piece1.GetComponent<Rigidbody>().isKinematic = false;
            piece1.GetComponent<ObjectManipulator>().enabled = false;

            childPose = assembled.transform.GetChild(2 * (assemblyStep - 1)).transform;
            pickPosition = piece1.transform.localPosition;
        }
        else
        {
            piece2.GetComponent<Rigidbody>().isKinematic = false;
            piece2.GetComponent<ObjectManipulator>().enabled = false;

            childPose = assembled.transform.GetChild(2 * (assemblyStep - 1) + 1).transform;
            pickPosition = piece2.transform.localPosition;
        }

        PlannerServiceRequest request = null;
        placePosition = ground.transform.InverseTransformPoint(childPose.transform.position);
        var noRot = 0;
        if(childPose.localRotation != Quaternion.Euler(0, 0, 0))
        {
            noRot = 1;
        }
        request = controller.PickAndPlaceService(
            noRot,
            pickPosition,
            placePosition
        );
        ros.SendServiceMessage<PlannerServiceResponse>(plannerServiceName, request, plannerCallback);
        while (controller.coroutineRunning)
        {
            yield return new WaitForSeconds(0.1f);
        }
        if(rnd == 1)
        {
            piece1.transform.SetPositionAndRotation(childPose.position, childPose.localRotation);
            piece1.GetComponent<Rigidbody>().isKinematic = true;
        }
        else
        {
            piece2.transform.SetPositionAndRotation(childPose.position, childPose.localRotation);
            piece2.GetComponent<Rigidbody>().isKinematic = true;
        }
    }

    private void plannerCallback(PlannerServiceResponse response)
    {
        StartCoroutine(controller.ExecuteTrajectories(response));
    }
}