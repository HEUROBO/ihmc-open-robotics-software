package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.Vector64F;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIKPacketCreator;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WholeBodyInverseKinematicBehavior extends BehaviorInterface
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);

   private final WholeBodyIKPacketCreator wholeBodyNetworkModule;
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final SDFFullRobotModel actualFullRobotModel;
   private final SDFFullRobotModel desiredFullRobotModel;
   private final ArrayList<Packet> packetsToSend = new ArrayList<Packet>();

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable trajectoryTime;
   private final BooleanYoVariable trajectoryTimeElapsed;
   private final BooleanYoVariable hasComputationBeenDone;
   private final BooleanYoVariable hasSolutionBeenFound;
   private RobotSide robotSide;

   private final boolean DEBUG = false;

   public WholeBodyInverseKinematicBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge,
         WholeBodyControllerParameters wholeBodyControllerParameters, SDFFullRobotModel actualFullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      wholeBodyNetworkModule = new WholeBodyIKPacketCreator(wholeBodyControllerParameters);
      wholeBodyIKSolver = wholeBodyControllerParameters.createWholeBodyIkSolver();

      this.yoTime = yoTime;

      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      startTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "StartTime", registry);
      startTime.set(Double.NaN);
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(Double.NaN);
      trajectoryTimeElapsed = new BooleanYoVariable(behaviorNameFirstLowerCase + "TrajectoryTimeElapsed", registry);
      hasComputationBeenDone= new BooleanYoVariable(behaviorNameFirstLowerCase + "hasComputationBeenDone", registry);
      hasSolutionBeenFound = new BooleanYoVariable(behaviorNameFirstLowerCase + "solutionHaveBeenFound", registry);

      this.actualFullRobotModel = actualFullRobotModel;
      this.desiredFullRobotModel = wholeBodyControllerParameters.createFullRobotModel();
   }

   public void setInputs(RobotSide robotSide, FramePose endEffectorPose, double trajectoryDuration)
   {
      wholeBodyIKSolver.setNumberOfControlledDoF(robotSide, ControlledDoF.DOF_3P3R);
      wholeBodyIKSolver.setNumberOfControlledDoF(robotSide.getOppositeSide(), ControlledDoF.DOF_NONE);
      trajectoryTime.set(trajectoryDuration);
      wholeBodyIKSolver.setGripperAttachmentTarget(actualFullRobotModel, robotSide, endEffectorPose);
      this.robotSide = robotSide;

      hasInputBeenSet.set(true);
   }

   public void computeSolution()
   {
      try
      {
         wholeBodyIKSolver.maxNumberOfAutomaticReseeds = 0;
         ComputeResult result = wholeBodyIKSolver.compute(actualFullRobotModel, desiredFullRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
         hasComputationBeenDone.set(true);
         if (result == ComputeResult.SUCCEEDED)
            hasSolutionBeenFound.set(true);
         else
         {
            Vector64F error = wholeBodyIKSolver.taskEndEffectorPose.get(robotSide).getError();
            Vector3d positionError = new Vector3d(error.get(0), error.get(1), error.get(2));
            Vector3d rotationError = new Vector3d(error.get(3), error.get(4), error.get(5));

            if (positionError.length() < 0.025 && rotationError.length() < 0.2)
            {
               hasSolutionBeenFound.set(true);
            }
            hasSolutionBeenFound.set(false);
         }
         if(DEBUG)
            System.out.println("solution found = " + hasSolutionBeenFound.getBooleanValue());
      }
      catch (Exception e)
      {
         System.out.println(e); // TODO: handle exception
      }   
   }
   
   @Override
   public void doControl()
   {
      //TODO check all the status 
      if (!packetHasBeenSent.getBooleanValue() && hasComputationBeenDone.getBooleanValue())
      {
         if (hasSolutionBeenFound.getBooleanValue())
         {
            sendSolutionToController(trajectoryTime.getDoubleValue());
            packetHasBeenSent.set(true);
         }
      }
   }

   private void sendSolutionToController(double trajectoryDuration)
   {
      packetsToSend.clear();
      startTime.set(yoTime.getDoubleValue());
      wholeBodyNetworkModule.createPackets(desiredFullRobotModel, trajectoryDuration, packetsToSend);
      for (int i = 0; i < packetsToSend.size(); i++)
      {
         sendPacketToController(packetsToSend.get(i));
      }
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);
      hasSolutionBeenFound.set(false);
      hasComputationBeenDone.set(false);

      isPaused.set(false);
      isStopped.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);

      wholeBodyIKSolver.setVerbosityLevel(0);
      wholeBodyIKSolver.getHierarchicalSolver().collisionAvoidance.setEnabled(true);
   }

   @Override
   public void finalize()
   {
      packetHasBeenSent.set(false);
      hasInputBeenSet.set(false);
      hasSolutionBeenFound.set(false);
      hasComputationBeenDone.set(false);

      isPaused.set(false);
      isStopped.set(false);

      startTime.set(Double.NaN);
      trajectoryTime.set(Double.NaN);
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(startTime.getDoubleValue()) || Double.isNaN(trajectoryTime.getDoubleValue()))
         trajectoryTimeElapsed.set(false);
      else
         trajectoryTimeElapsed.set(yoTime.getDoubleValue() - startTime.getDoubleValue() > trajectoryTime.getDoubleValue());

      return (trajectoryTimeElapsed.getBooleanValue() && !isPaused.getBooleanValue() ) || (hasComputationBeenDone.getBooleanValue() && !hasSolutionBeenFound());
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
 
   }
   
   public boolean hasSolutionBeenFound()
   {
      return hasSolutionBeenFound.getBooleanValue();
   }
}
