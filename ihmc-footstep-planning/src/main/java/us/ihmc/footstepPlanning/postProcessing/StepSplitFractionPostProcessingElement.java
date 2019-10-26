package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;

import java.util.List;

public class StepSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private final FootstepPostProcessingParametersReadOnly parameters;
   private final ICPPlannerParameters walkingControllerParameters;

   public StepSplitFractionPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters,
                                                 ICPPlannerParameters walkingControllerParameters)
   {
      this.parameters = parameters;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return parameters.positionSplitFractionProcessingEnabled();
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPlanningToolboxOutputStatus postProcessFootstepPlan(FootstepPlanningRequestPacket request, FootstepPlanningToolboxOutputStatus outputStatus)
   {
      FootstepPlanningToolboxOutputStatus processedOutput = new FootstepPlanningToolboxOutputStatus(outputStatus);

      FramePose3D stanceFootPose = new FramePose3D();
      stanceFootPose.setPosition(request.getStanceFootPositionInWorld());
      stanceFootPose.setOrientation(request.getStanceFootOrientationInWorld());

      FramePose3D nextFootPose = new FramePose3D();

      double defaultTransferSplitFraction = walkingControllerParameters.getTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      List<FootstepDataMessage> footstepDataMessageList = processedOutput.getFootstepDataList().getFootstepDataList();
      for (int stepNumber = 0; stepNumber < footstepDataMessageList.size(); stepNumber++)
      {
         if (stepNumber > 0)
         {
            stanceFootPose.setPosition(footstepDataMessageList.get(stepNumber - 1).getLocation());
            stanceFootPose.setOrientation(footstepDataMessageList.get(stepNumber - 1).getOrientation());
         }

         nextFootPose.setPosition(footstepDataMessageList.get(stepNumber).getLocation());
         nextFootPose.setOrientation(footstepDataMessageList.get(stepNumber).getOrientation());

         // This step is a big step down.
         double stepDownHeight = nextFootPose.getZ() - stanceFootPose.getZ();

         if (nextFootPose.getZ() - stanceFootPose.getZ() < -parameters.getStepHeightForLargeStepDown())
         {
            double alpha = Math.min(1.0, Math.abs(stepDownHeight) / parameters.getLargestStepDownHeight());
            double transferSplitFraction = InterpolationTools.linearInterpolate(defaultTransferSplitFraction,
                                                                                parameters.getTransferSplitFractionAtFullDepth(), alpha);
            double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                     parameters.getTransferWeightDistributionAtFullDepth(), alpha);

            if (stepNumber == footstepDataMessageList.size() - 1)
            { // this is the last step
               double currentSplitFraction = processedOutput.getFootstepDataList().getFinalTransferSplitFraction();
               double currentWeightDistribution = processedOutput.getFootstepDataList().getFinalTransferWeightDistribution();

               double splitFractionToSet, weightDistributionToSet;

               if (currentSplitFraction == -1.0)
                  splitFractionToSet = transferSplitFraction;
               else
                  splitFractionToSet = transferSplitFraction * currentSplitFraction / defaultTransferSplitFraction;

               if (currentWeightDistribution == -1.0)
                  weightDistributionToSet = transferWeightDistribution;
               else
                  weightDistributionToSet = transferWeightDistribution * currentWeightDistribution / defaultWeightDistribution;

               processedOutput.getFootstepDataList().setFinalTransferSplitFraction(splitFractionToSet);
               processedOutput.getFootstepDataList().setFinalTransferWeightDistribution(weightDistributionToSet);
            }
            else
            {
               double currentSplitFraction = footstepDataMessageList.get(stepNumber + 1).getTransferSplitFraction();
               double currentWeightDistribution = footstepDataMessageList.get(stepNumber + 1).getTransferWeightDistribution();

               double splitFractionToSet, weightDistributionToSet;

               if (currentSplitFraction == -1.0)
                  splitFractionToSet = transferSplitFraction;
               else
                  splitFractionToSet = transferSplitFraction * currentSplitFraction / defaultTransferSplitFraction;

               if (currentWeightDistribution == -1.0)
                  weightDistributionToSet = transferWeightDistribution;
               else
                  weightDistributionToSet = transferWeightDistribution * currentWeightDistribution / defaultWeightDistribution;

               footstepDataMessageList.get(stepNumber + 1).setTransferSplitFraction(splitFractionToSet);
               footstepDataMessageList.get(stepNumber + 1).setTransferWeightDistribution(weightDistributionToSet);
            }
         }


      }

      return processedOutput;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.STEP_SPLIT_FRACTIONS;
   }
}
