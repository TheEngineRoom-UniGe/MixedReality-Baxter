using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.PandaUnity;
using RosMessageTypes.Std;
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

    // Canvas and interface objects
    public GameObject poseMarker;
    private GameObject[] buttons;

    // Variables required for ROS communication
    public string plannerServiceName = "motion_planner_service";
    public string nextStepTopicName = "next_step";
    public string stepDoneTopicName = "step_done";
    public string collisionCountTopicName = "collision_count";

    private RobotController controller;
    private RobotController controller_hologram;
    private bool isHoloActive = false;

    // Utility variables
    private int nChildren;
    private int assemblyStep = 0;
    private int assemblyCount = 0;
    private int stepSolvedCount = 0;
    private int collisionCount = 0;
    private bool prevStepSolved = true;
    private Vector3 markerPosition;

    private List<Vector3> piece1InitPositions;
    private List<Vector3> piece2InitPositions;
    private List<Quaternion> piece1InitRotations;
    private List<Quaternion> piece2InitRotations;

    private Material green;
    private Material wood;

    //private Vector3[] buttonsOffsets = { new Vector3(-0.6f, 0.65f, 0.0f), new Vector3(0.6f, 0.65f, 0.0f), new Vector3(0.6f, 0.5f, 0.0f) };

    public float deltaT = 10.0f;


    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Subscribe to next step message
        ros.Subscribe<Int32>(nextStepTopicName, Dispatcher);

        // Instantiate Robot Controller
        controller = gameObject.AddComponent<RobotController>();
        controller.Init(robot, ground.transform);

        // Instantiate Holographic Robot  Controller
        controller_hologram = gameObject.AddComponent<RobotController>();
        controller_hologram.Init(robot_hologram, ground_holo.transform);

        nChildren = chair.transform.childCount;

        piece1InitPositions = new List<Vector3>();
        piece2InitPositions = new List<Vector3>();
        piece1InitRotations = new List<Quaternion>();
        piece2InitRotations = new List<Quaternion>();

        wood = Resources.Load("Materials/wood") as Material;
        green = Resources.Load("Materials/green") as Material;
    }

    public void Spawn()
    {
        markerPosition = new Vector3(
            poseMarker.transform.position.x,
            poseMarker.transform.position.y, 
            poseMarker.transform.position.z
        );
        poseMarker.GetComponent<Behaviour>().enabled = false;

        controller.Spawn(markerPosition);
        
        poseMarker.SetActive(false);
    }

    private void Dispatcher(Int32 msg)
    {
        if(msg.data > 0 && msg.data < 6)
        {
            NextAssemblyStep(msg.data);
            return;
        }
        else if(msg.data > 6)
        {
            if(msg.data == 8)
            {
                SetHologram(true);
            }
            else
            {
                SetHologram(false);
            }
        }
        else if(msg.data == -1)
        {
            ResetStep();
            return;
        }
        else if(msg. data == 0)
        {
            ResetScene();
            return;
        }
    }

    private void NextAssemblyStep(int value)
    {
        if(assemblyCount < nChildren - 1 && prevStepSolved)
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

            piece1InitPositions.Add(new Vector3(piece1.transform.position.x, piece1.transform.position.y, piece1.transform.position.z));
            piece2InitPositions.Add(new Vector3(piece2.transform.position.x, piece2.transform.position.y, piece2.transform.position.z));

            piece1InitRotations.Add(new Quaternion(
                piece1.transform.rotation.x,
                piece1.transform.rotation.y,
                piece1.transform.rotation.z,
                piece1.transform.rotation.w
            ));
            piece2InitRotations.Add(new Quaternion(
                piece2.transform.rotation.x,
                piece2.transform.rotation.y,
                piece2.transform.rotation.z,
                piece2.transform.rotation.w
            ));

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
        }

        if(value < 5)
        {
            StartCoroutine(SolveStep());
        }  
    }

    private void ResetStep()
    {
        piece1.transform.SetPositionAndRotation(piece1InitPositions[assemblyStep-1], piece1InitRotations[assemblyStep-1]);
        piece1.GetComponent<ObjectManipulator>().enabled = true;
        piece1.GetComponent<Renderer>().material = wood;

        piece2.transform.SetPositionAndRotation(piece2InitPositions[assemblyStep-1], piece2InitRotations[assemblyStep-1]);
        piece2.GetComponent<ObjectManipulator>().enabled = true;
        piece2.GetComponent<Renderer>().material = wood;

        stepSolvedCount = 0;

        StartCoroutine(SolveStep());
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
                piece1.GetComponent<Renderer>().material = green;
                
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
                piece2.GetComponent<Renderer>().material = green;

                stepSolvedCount += 1;
            }
        }

        if(stepSolvedCount == 2)
        {
            stepSolvedCount = 0;
            prevStepSolved = true;
            ros.Send(stepDoneTopicName, new Bool(data: true));
        }
    }

    private IEnumerator SolveStep()
    {
        yield return new WaitForSeconds(1.0f);
        StartCoroutine(PlanAction());
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

            childPose = assembled.transform.GetChild(2 * (assemblyStep - 1)).transform;
            pickPosition = piece1.transform.localPosition;
        }
        else
        {
            piece2.GetComponent<Rigidbody>().isKinematic = false;

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
        if(rnd == 1 && (piece1.transform.position - childPose.transform.position).magnitude < 0.05f)
        {
            
            piece1.transform.SetPositionAndRotation(childPose.position, childPose.localRotation);
            piece1.GetComponent<Rigidbody>().isKinematic = true;
            piece1.GetComponent<ObjectManipulator>().enabled = false;
            piece1.GetComponent<Renderer>().material = green;
            stepSolvedCount += 1;
        }
        else if(rnd == 1 && (piece1.transform.position - childPose.transform.position).magnitude >= 0.05f)
        {
            piece1.GetComponent<Rigidbody>().isKinematic = true;
        }
        else if(rnd == 2 && (piece2.transform.position - childPose.transform.position).magnitude < 0.05f)
        {
            piece2.transform.SetPositionAndRotation(childPose.position, childPose.localRotation);
            piece2.GetComponent<Rigidbody>().isKinematic = true;
            piece2.GetComponent<ObjectManipulator>().enabled = false;
            piece2.GetComponent<Renderer>().material = green;
            stepSolvedCount += 1;
        }
        else if(rnd == 2 && (piece2.transform.position - childPose.transform.position).magnitude >= 0.05f)
        {
            piece2.GetComponent<Rigidbody>().isKinematic = true;
        }

        if(stepSolvedCount == 2)
        {
            stepSolvedCount = 0;
            prevStepSolved = true;
            ros.Send(stepDoneTopicName, new Bool(data: true));
        }
    }

    private void plannerCallback(PlannerServiceResponse response)
    {
        StartCoroutine(TrajectoryExecutioner(response));
    }

    private IEnumerator TrajectoryExecutioner(PlannerServiceResponse response)
    {
        if(isHoloActive)
        {
            StartCoroutine(controller_hologram.ExecuteTrajectories(response));
            
        }
        yield return new WaitForSeconds(deltaT);
        StartCoroutine(controller.ExecuteTrajectories(response));
    }

    private void SetHologram(bool value)
    {
        if(!isHoloActive && value)
        {
            isHoloActive = true;
            controller_hologram.Spawn(robot.transform.position);
        }
        else if(isHoloActive && !value)
        {
            isHoloActive = false;
            robot_hologram.SetActive(false);
        }
        
    }

    private void ResetScene()
    {
        int aux = 0;
        for(int i=0; i< piece1InitPositions.Count; i++)
        {
            var piece1 = chair.transform.GetChild(aux);
            var piece2 = chair.transform.GetChild(aux+1);

            var c1 = piece1.GetComponent<CollisionCheck>();
            var c2 = piece2.GetComponent<CollisionCheck>();

            collisionCount += (c1.GetCollisions() + c2.GetCollisions());

            c1.ResetCollisions();
            c2.ResetCollisions();

            piece1.SetPositionAndRotation(piece1InitPositions[i], piece1InitRotations[i]);
            piece2.SetPositionAndRotation(piece2InitPositions[i], piece2InitRotations[i]);
            piece1.GetComponent<ObjectManipulator>().enabled = true;
            piece2.GetComponent<ObjectManipulator>().enabled = true;
            piece1.GetComponent<Renderer>().material = wood;
            piece2.GetComponent<Renderer>().material = wood;

            aux +=3;
        }
        ros.Send(collisionCountTopicName, new Int32(data: collisionCount));

        assembled.SetActive(false);
        for(int i=0; i < assemblyCount; i++) {
            chair.transform.GetChild(i).gameObject.SetActive(false);
        }

        assemblyCount = 0;
        assemblyStep = 0;
        stepSolvedCount = 0;
        collisionCount = 0;
        prevStepSolved = true;

        piece1InitPositions.Clear();
        piece2InitPositions.Clear();
        piece1InitRotations.Clear();
        piece2InitRotations.Clear();
    }
}