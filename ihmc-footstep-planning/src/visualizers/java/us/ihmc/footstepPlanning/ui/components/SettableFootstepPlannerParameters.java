package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class SettableFootstepPlannerParameters implements FootstepPlannerParameters
{
   private double idealFootstepWidth;
   private double idealFootstepLength;

   private double maxStepReach;
   private double maxStepYaw;
   private double minStepWidth;
   private double minStepLength;
   private double minStepYaw;
   private double maxStepZ;
   private double minFootholdPercent;
   private double minSurfaceIncline;
   private double maxStepWidth;

   private double yawWeight;
   private double costPerStep;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlanningParameters)
   {
      this.idealFootstepWidth = footstepPlanningParameters.getIdealFootstepWidth();
      this.idealFootstepLength = footstepPlanningParameters.getIdealFootstepLength();
      this.maxStepReach = footstepPlanningParameters.getMaximumStepReach();
      this.maxStepYaw = footstepPlanningParameters.getMaximumStepYaw();
      this.minStepWidth = footstepPlanningParameters.getMinimumStepWidth();
      this.minStepLength = footstepPlanningParameters.getMinimumStepLength();
      this.minStepYaw = footstepPlanningParameters.getMinimumStepYaw();
      this.maxStepZ = footstepPlanningParameters.getMaximumStepZ();
      this.maxStepWidth = footstepPlanningParameters.getMaximumStepWidth();
      this.minFootholdPercent = footstepPlanningParameters.getMinimumFootholdPercent();
      this.minSurfaceIncline = footstepPlanningParameters.getMinimumSurfaceInclineRadians();

      this.yawWeight = footstepPlanningParameters.getCostParameters().getYawWeight();
      this.costPerStep = footstepPlanningParameters.getCostParameters().getCostPerStep();
   }

   public void setIdealFootstepWidth(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setMaximumStepReach(double maxStepReach)
   {
      this.maxStepReach = maxStepReach;
   }

   public void setMaximumStepYaw(double maxStepYaw)
   {
      this.maxStepYaw = maxStepYaw;
   }

   public void setMinimumStepWidth(double minStepWidth)
   {
      this.minStepWidth = minStepWidth;
   }

   public void setMinimumStepLength(double minStepLength)
   {
      this.minStepLength = minStepLength;
   }

   public void setMinimumStepYaw(double minStepYaw)
   {
      this.minStepYaw = minStepYaw;
   }

   public void setMaximumStepZ(double maxStepZ)
   {
      this.maxStepZ = maxStepZ;
   }

   public void setMaximumStepWidth(double maxStepWidth)
   {
      this.maxStepWidth = maxStepWidth;
   }

   public void setMinimumFootholdPercent(double minFootholdPercent)
   {
      this.minFootholdPercent = minFootholdPercent;
   }

   public void setMinimumSurfaceInclineRadians(double minSurfaceIncline)
   {
      this.minSurfaceIncline = minSurfaceIncline;
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength;
   }

   @Override
   public double getMaximumStepReach()
   {
      return maxStepReach;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maxStepYaw;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minStepWidth;
   }

   @Override
   public double getMinimumStepLength()
   {
      return minStepLength;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minStepYaw;
   }

   @Override
   public double getMaximumStepZ()
   {
      return maxStepZ;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maxStepWidth;
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minFootholdPercent;
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minSurfaceIncline;
   }

   @Override
   public double getYawWeight()
   {
      return yawWeight;
   }

   @Override
   public double getCostPerStep()
   {
      return costPerStep;
   }
}
