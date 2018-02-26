package us.ihmc.humanoidRobotics.communication.packets.bdi;

import us.ihmc.communication.packets.Packet;

public class BDIBehaviorCommandPacket extends Packet<BDIBehaviorCommandPacket>
{
   public BDIRobotBehavior atlasRobotBehavior;
   public boolean stop = false;

   public BDIBehaviorCommandPacket()
   {
   }

   @Override
   public void set(BDIBehaviorCommandPacket other)
   {
      setPacketInformation(other);
      atlasRobotBehavior = other.atlasRobotBehavior;
      stop = other.stop;
   }

   @Override
   public boolean equals(Object other)
   {
      return (other instanceof BDIBehaviorCommandPacket) && epsilonEquals((BDIBehaviorCommandPacket) other, 0);
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorCommandPacket other, double epsilon)
   {
      return (other.atlasRobotBehavior == atlasRobotBehavior) && (other.stop == stop);
   }
}
