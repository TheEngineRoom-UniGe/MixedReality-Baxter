//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.BaxterUnity
{
    [Serializable]
    public class ActionServiceResponse : Message
    {
        public const string k_RosMessageName = "baxter_unity/ActionService";
        public override string RosMessageName => k_RosMessageName;

        public int seq;
        public string action;
        public PlannedTrajectoryMsg arm_trajectory;

        public ActionServiceResponse()
        {
            this.seq = 0;
            this.action = "";
            this.arm_trajectory = new PlannedTrajectoryMsg();
        }

        public ActionServiceResponse(int seq, string action, PlannedTrajectoryMsg arm_trajectory)
        {
            this.seq = seq;
            this.action = action;
            this.arm_trajectory = arm_trajectory;
        }

        public static ActionServiceResponse Deserialize(MessageDeserializer deserializer) => new ActionServiceResponse(deserializer);

        private ActionServiceResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.seq);
            deserializer.Read(out this.action);
            this.arm_trajectory = PlannedTrajectoryMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.seq);
            serializer.Write(this.action);
            serializer.Write(this.arm_trajectory);
        }

        public override string ToString()
        {
            return "ActionServiceResponse: " +
            "\nseq: " + seq.ToString() +
            "\naction: " + action.ToString() +
            "\narm_trajectory: " + arm_trajectory.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}