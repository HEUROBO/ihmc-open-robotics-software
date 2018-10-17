package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class FootstepPlannerParametersProperty extends ParametersProperty<SettableFootstepPlannerParameters>
{
   private DoubleField idealFootstepWidth = new DoubleField(SettableFootstepPlannerParameters::getIdealFootstepWidth, (p, v) -> p.setIdealFootstepWidth(v));
   private DoubleField idealFootstepLength = new DoubleField(SettableFootstepPlannerParameters::getIdealFootstepLength, (p, v) -> p.setIdealFootstepLength(v));
   private DoubleField maxStepReach = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepReach, (p, v) -> p.setMaximumStepReach(v));
   private DoubleField maxStepYaw = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepYaw, (p, v) -> p.setMaximumStepYaw(v));
   private DoubleField minStepWidth = new DoubleField(SettableFootstepPlannerParameters::getMinimumStepWidth, (p, v) -> p.setMinimumStepWidth(v));
   private DoubleField minStepLength = new DoubleField(SettableFootstepPlannerParameters::getMinimumStepLength, (p, v) -> p.setMinimumStepLength(v));
   private DoubleField minStepYaw = new DoubleField(SettableFootstepPlannerParameters::getMinimumStepYaw, (p, v) -> p.setMinimumStepYaw(v));
   private DoubleField maxStepZ = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepZ, (p, v) -> p.setMaximumStepZ(v));
   private DoubleField minFootholdPercent = new DoubleField(SettableFootstepPlannerParameters::getMinimumFootholdPercent, (p, v) -> p.setMinimumFootholdPercent(v));
   private DoubleField minSurfaceIncline = new DoubleField(SettableFootstepPlannerParameters::getMinimumSurfaceInclineRadians, (p, v) -> p.setMinimumSurfaceInclineRadians(v));
   private DoubleField maxStepWidth = new DoubleField(SettableFootstepPlannerParameters::getMaximumStepWidth, (p, v) -> p.setMaximumStepWidth(v));

   private DoubleField yawWeight = new DoubleField(SettableFootstepPlannerParameters::getYawWeight, (p, v) -> p.setYawWeight(v));
   private DoubleField costPerStep = new DoubleField(SettableFootstepPlannerParameters::getCostPerStep, (p, v) -> p.setCostPerStep(v));

   public FootstepPlannerParametersProperty(Object bean, String name)
   {
      super(bean, name, new SettableFootstepPlannerParameters(new DefaultFootstepPlanningParameters()));
   }

   @Override
   protected SettableFootstepPlannerParameters getValueCopy(SettableFootstepPlannerParameters valueToCopy)
   {
      return new SettableFootstepPlannerParameters(valueToCopy);
   }

   public void bidirectionalBindIdealFootstepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, idealFootstepWidth);
   }

   public void bidirectionalBindIdealFootstepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, idealFootstepLength);
   }

   public void bidirectionalBindMaxStepReach(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepReach);
   }

   public void bidirectionalBindMaxStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepYaw);
   }

   public void bidirectionalBindMinStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepWidth);
   }

   public void bidirectionalBindMinStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepLength);
   }

   public void bidirectionalBindMinStepYaw(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepYaw);
   }

   public void bidirectionalBindMaxStepZ(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepZ);
   }

   public void bidirectionalBindMinFootholdPercent(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minFootholdPercent);
   }

   public void bidirectionalBindMinSurfaceIncline(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minSurfaceIncline);
   }

   public void bidirectionalBindMaxStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepWidth);
   }

   public void bidirectionalBindYawWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yawWeight);
   }

   public void bidirectionalBindCostPerStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, costPerStep);
   }
}
