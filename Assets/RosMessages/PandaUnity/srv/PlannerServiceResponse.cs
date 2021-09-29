//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.PandaUnity
{
    public class PlannerServiceResponse : Message
    {
        public const string RosMessageName = "panda_unity/PlannerService";

        public ArmTrajectory arm_trajectory;

        public PlannerServiceResponse()
        {
            this.arm_trajectory = new ArmTrajectory();
        }

        public PlannerServiceResponse(ArmTrajectory arm_trajectory)
        {
            this.arm_trajectory = arm_trajectory;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(arm_trajectory.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.arm_trajectory.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "PlannerServiceResponse: " +
            "\narm_trajectory: " + arm_trajectory.ToString();
        }
    }
}
