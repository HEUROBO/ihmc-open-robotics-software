package us.ihmc.quadrupedFootstepPlanning.ui.components;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;

public class SettableFootstepPlannerParameters implements FootstepPlannerParametersBasics
{
   private double maximumFrontStepReach;
   private double maximumFrontStepLength;
   private double minimumFrontStepLength;
   private double maximumHindStepReach;
   private double maximumHindStepLength;
   private double minimumHindStepLength;

   private double maximumStepWidth;
   private double minimumStepWidth;
   private double minimumStepYaw;
   private double maximumStepYaw;

   private double maximumStepChangeZ;
   private double bodyGroundClearance;
   private double maxWalkingSpeedMultiplier;

   private double yawWeight;
   private double costPerStep;
   private double stepUpWeight;
   private double stepDownWeight;
   private double distanceWeight;
   private double heuristicsWeight;
   private double xGaitWeight;

   private double minXClearanceFromFoot;
   private double minYClearanceFromFoot;
   private double minimumSurfaceInclineRadians;
   private double projectInsideDistanceForExpansion;
   private double projectInsideDistanceForPostProcessing;
   private double maximumXYWiggleDistance;
   private double cliffHeightToAvoid;
   private double minimumDistanceFromCliffTops;
   private double minimumDistanceFromCliffBottoms;

   private boolean projectInsideUsingConvexHull;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumFrontStepReach(double maximumStepReach)
   {
      this.maximumFrontStepReach = maximumStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumFrontStepLength(double maximumStepLength)
   {
      this.maximumFrontStepLength = maximumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumFrontStepLength(double minimumStepLength)
   {
      this.minimumFrontStepLength = minimumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumHindStepReach(double maximumStepReach)
   {
      this.maximumHindStepReach = maximumStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumHindStepLength(double maximumStepLength)
   {
      this.maximumHindStepLength = maximumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumHindStepLength(double minimumStepLength)
   {
      this.minimumHindStepLength = minimumStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth = maximumStepWidth;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth = minimumStepWidth;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumStepYaw(double minimumStepYaw)
   {
      this.minimumStepYaw = minimumStepYaw;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw = maximumStepYaw;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumStepChangeZ(double maximumStepChangeZ)
   {
      this.maximumStepChangeZ = maximumStepChangeZ;
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance = bodyGroundClearance;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaxWalkingSpeedMultiplier(double maxWalkingSpeedMultiplier)
   {
      this.maxWalkingSpeedMultiplier = maxWalkingSpeedMultiplier;
   }

   @Override
   public void setDistanceHeuristicWeight(double distanceHeuristicWeight)
   {
      this.distanceWeight = distanceHeuristicWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setYawWeight(double yawWeight)
   {
      this.yawWeight = yawWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setXGaitWeight(double xGaitWeight)
   {
      this.xGaitWeight = xGaitWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep = costPerStep;
   }

   /** {@inheritDoc} */
   @Override
   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight = stepUpWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight = stepDownWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setHeuristicsInflationWeight(double heuristicsWeight)
   {
      this.heuristicsWeight = heuristicsWeight;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinXClearanceFromFoot(double minXClearanceFromFoot)
   {
      this.minXClearanceFromFoot = minXClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinYClearanceFromFoot(double minYClearanceFromFoot)
   {
      this.minYClearanceFromFoot = minYClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public void setMinimumSurfaceInclineRadians(double minimumSurfaceInclineRadians)
   {
      this.minimumSurfaceInclineRadians = minimumSurfaceInclineRadians;
   }

   @Override
   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid = cliffHeightToAvoid;
   }

   @Override
   public void setMinimumDistanceFromCliffBottoms(double distance)
   {
      this.minimumDistanceFromCliffBottoms = distance;
   }

   @Override
   public void setMinimumDistanceFromCliffTops(double distance)
   {
      this.minimumDistanceFromCliffTops = distance;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideDistanceForExpansion(double projectInsideDistance)
   {
      this.projectInsideDistanceForExpansion = projectInsideDistance;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideDistanceForPostProcessing(double projectInsideDistance)
   {
      this.projectInsideDistanceForPostProcessing = projectInsideDistance;
   }

   /** {@inheritDoc} */
   @Override
   public void setProjectInsideUsingConvexHull(boolean projectInsideUsingConvexHull)
   {
      this.projectInsideUsingConvexHull = projectInsideUsingConvexHull;
   }

   /** {@inheritDoc} */
   @Override
   public void setMaximumXYWiggleDistance(double maximumWiggleDistance)
   {
      this.maximumXYWiggleDistance = maximumWiggleDistance;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepReach()
   {
      return maximumFrontStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumFrontStepLength()
   {
      return maximumFrontStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumFrontStepLength()
   {
      return minimumFrontStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepReach()
   {
      return maximumHindStepReach;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumHindStepLength()
   {
      return maximumHindStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumHindStepLength()
   {
      return minimumHindStepLength;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepWidth()
   {
      return maximumStepWidth;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepWidth()
   {
      return minimumStepWidth;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumStepYaw()
   {
      return minimumStepYaw;
   }

   /** {@inheritDoc} */
   public double getMaximumStepYaw()
   {
      return maximumStepYaw;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumStepChangeZ()
   {
      return maximumStepChangeZ;
   }

   /** {@inheritDoc} */
   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxWalkingSpeedMultiplier()
   {
      return maxWalkingSpeedMultiplier;
   }

   @Override
   public double getDistanceHeuristicWeight()
   {
      return distanceWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getXGaitWeight()
   {
      return xGaitWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getYawWeight()
   {
      return yawWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getCostPerStep()
   {
      return costPerStep;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepUpWeight()
   {
      return stepUpWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepDownWeight()
   {
      return stepDownWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getHeuristicsInflationWeight()
   {
      return heuristicsWeight;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinXClearanceFromFoot()
   {
      return minXClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinYClearanceFromFoot()
   {
      return minYClearanceFromFoot;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForExpansion()
   {
      return projectInsideDistanceForExpansion;
   }

   /** {@inheritDoc} */
   @Override
   public double getProjectInsideDistanceForPostProcessing()
   {
      return projectInsideDistanceForPostProcessing;
   }

   /** {@inheritDoc} */
   @Override
   public boolean getProjectInsideUsingConvexHull()
   {
      return projectInsideUsingConvexHull;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance;
   }

   /** {@inheritDoc} */
   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinimumDistanceFromCliffTops()
   {
      return minimumDistanceFromCliffTops;
   }
}
