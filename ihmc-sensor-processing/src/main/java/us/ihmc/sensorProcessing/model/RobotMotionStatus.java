package us.ihmc.sensorProcessing.model;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum RobotMotionStatus
{
   @RosEnumValueDocumentation(documentation = "It is uncertain if the robot is moving or not, it can only be determined when the walking controller is active.")
   UNKNOWN(2),
   @RosEnumValueDocumentation(documentation = "Robot is standing")
   STANDING(3),
   @RosEnumValueDocumentation(documentation = "Robot is in motion")
   IN_MOTION(4);

   public int behaviorId;

   private RobotMotionStatus(int behaviorId)
   {
      this.behaviorId = behaviorId;
   }

   public int getBehaviorId()
   {
      return behaviorId;
   }

   public static RobotMotionStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static RobotMotionStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
