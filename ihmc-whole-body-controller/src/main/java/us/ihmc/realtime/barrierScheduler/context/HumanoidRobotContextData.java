package us.ihmc.realtime.barrierScheduler.context;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

import java.util.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextData implements InPlaceCopyable<HumanoidRobotContextData>
{
   private final HumanoidRobotContextJointData rawJointData;
   private final HumanoidRobotContextJointData processedJointData;

   private final ForceSensorDataHolder forceSensorDataHolder;
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final RobotMotionStatusHolder robotMotionStatusHolder;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final ArrayList<RigidBodyBasics> robotFeet;
   private final Map<String, Point2D> copPoints = new HashMap<>();

   HumanoidRobotContextData(FullHumanoidRobotModel fullRobotModel, HumanoidRobotContextJointData rawJointData, HumanoidRobotContextJointData processedJointData,
                            ForceSensorDataHolder forceSensorDataHolder, CenterOfPressureDataHolder centerOfPressureDataHolder, RobotMotionStatusHolder robotMotionStatusHolder,
                            JointDesiredOutputList jointDesiredOutputList)
   {
      this.rawJointData = rawJointData;
      this.processedJointData = processedJointData;
      this.forceSensorDataHolder = forceSensorDataHolder;

      this.robotFeet = new ArrayList<>(centerOfPressureDataHolder.getRigidBodies());

      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      this.robotMotionStatusHolder = robotMotionStatusHolder;
      this.jointDesiredOutputList = jointDesiredOutputList;

      for (int i = 0; i < robotFeet.size(); i++)
      {
         RigidBodyBasics foot = robotFeet.get(i);
         copPoints.put(foot.getName(), new Point2D());
      }
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      this.rawJointData.copyFrom(src.rawJointData);
      this.processedJointData.copyFrom(src.processedJointData);

      this.forceSensorDataHolder.set(src.forceSensorDataHolder);

      for (int i = 0; i < robotFeet.size(); i++)
      {
         RigidBodyBasics foot = robotFeet.get(i);
         Point2D copTmp = copPoints.get(foot.getName());

         src.centerOfPressureDataHolder.getCenterOfPressureByName(copTmp, foot);
         this.centerOfPressureDataHolder.setCenterOfPressureByName(copTmp, foot);
      }

      this.robotMotionStatusHolder.setCurrentRobotMotionStatus(src.robotMotionStatusHolder.getCurrentRobotMotionStatus());

      for (int i = 0; i < this.jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         this.jointDesiredOutputList.getJointDesiredOutput(i).set(src.jointDesiredOutputList.getJointDesiredOutput(i));
      }
   }
}
