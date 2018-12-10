package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.utils.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.ListSorter;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class NewQuadrupedContactSequence extends RecyclingArrayList<NewQuadrupedContactPhase>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private Comparator<QuadrupedStepTransition> compareByTime = Comparator.comparingDouble(QuadrupedStepTransition::getTransitionTime);

   // internal states
   private final int pastContactPhaseCapacity;
   private final RecyclingArrayList<QuadrupedStepTransition> stepTransitions = new RecyclingArrayList<>(QuadrupedStepTransition::new);
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();


   private final int maxCapacity;

   private final QuadrantDependentList<ReferenceFrame> soleFrames;

   public NewQuadrupedContactSequence(QuadrantDependentList<ReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      super(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, NewQuadrupedContactPhase.class);
      clear();

      this.soleFrames = soleFrames;

      maxCapacity = pastContactPhaseCapacity + futureContactPhaseCapacity + 1;
      this.pastContactPhaseCapacity = pastContactPhaseCapacity;

      if (maxCapacity < 1)
      {
         throw new RuntimeException("Sequence capacity must be greater than zero.");
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetInContact.add(robotQuadrant);
         solePositions.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void initialize()
   {
      clear();
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param stepSequence list of upcoming steps (input)
    * @param currentFeetInContact list of current feet in contact (input)
    * @param currentTime current time (input)
    */
   public void update(List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositions.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         solePositions.get(robotQuadrant).changeFrame(worldFrame);
      }
      feetInContact.clear();
      for (int footIndex = 0; footIndex < currentFeetInContact.size(); footIndex++)
         feetInContact.add(currentFeetInContact.get(footIndex));

      // initialize step transitions
      for (int i = 0; i < stepTransitions.size(); i++)
      {
         stepTransitions.get(i).setTransitionTime(Double.MAX_VALUE);
      }

      // FIXME we might have some bugs in here
      int numberOfStepTransitions = 0;
      stepTransitions.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            QuadrupedStepTransition stepTransition = stepTransitions.add();

            stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());
            stepTransition.setTransitionType(QuadrupedStepTransitionType.LIFT_OFF);
            stepTransition.setRobotQuadrant(step.getRobotQuadrant());
            stepTransition.setSolePosition(stepTransition.getSolePosition());

            numberOfStepTransitions++;
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            QuadrupedStepTransition stepTransition = stepTransitions.add();

            stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
            stepTransition.setTransitionType(QuadrupedStepTransitionType.TOUCH_DOWN);
            stepTransition.setRobotQuadrant(step.getRobotQuadrant());
            stepTransition.setSolePosition(step.getGoalPosition());

            numberOfStepTransitions++;
         }
      }

      // sort step transitions in ascending order as a function of time
      ListSorter.sort(stepTransitions, compareByTime);

      // retain desired number of past contact phases
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, this);
      while (size() > pastContactPhaseCapacity + 1)
      {
         remove(0);
      }

      NewQuadrupedContactPhase contactPhase;
      if (isEmpty())
      {
         contactPhase = createNewContactPhase(currentTime, feetInContact, solePositions);
      }
      else
      {
         NewQuadrupedContactPhase lastContactPhase = get(size() - 1);
         if (isEqualContactState(lastContactPhase.getFeetInContact(), currentFeetInContact))
         {
            // extend current contact phase
            contactPhase = lastContactPhase;
            contactPhase.setSolePosition(solePositions);
         }
         else
         {
            // end previous contact phase
            lastContactPhase.getTimeInterval().setEndTime(currentTime);
            remove(0);

            contactPhase = createNewContactPhase(currentTime, feetInContact, solePositions);
         }
      }

      // compute transition time and center of pressure for each time interval
      for (int i = 0; i < numberOfStepTransitions; i++)
      {
         QuadrupedStepTransition stepTransition = stepTransitions.get(i);
         switch (stepTransitions.get(i).getTransitionType())
         {
         case LIFT_OFF:
            feetInContact.remove(stepTransition.getRobotQuadrant());
            break;
         case TOUCH_DOWN:
            feetInContact.add(stepTransition.getRobotQuadrant());
            solePositions.get(stepTransition.getRobotQuadrant()).changeFrame(worldFrame);
            solePositions.get(stepTransition.getRobotQuadrant()).set(stepTransition.getSolePosition());
            break;
         }

         if (i > stepTransitions.size() - 1)
            stepTransitions.add(new QuadrupedStepTransition());

         if ((i + 1 == numberOfStepTransitions) || (stepTransitions.get(i).getTransitionTime() != stepTransitions.get(i + 1).getTransitionTime()))
         {
            contactPhase.getTimeInterval().setEndTime(stepTransitions.get(i).getTransitionTime());

            contactPhase = createNewContactPhase(stepTransitions.get(i).getTransitionTime(), feetInContact, solePositions);
            if (contactPhase == null) // made the full sequence
            {
               return;
            }
         }
      }
      contactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getStartTime() + 1.0);
   }

   private boolean isEqualContactState(List<RobotQuadrant> contactStateA, List<RobotQuadrant> contactStateB)
   {
      if (contactStateA.size() != contactStateB.size())
         return false;

      for (int i = 0; i < contactStateA.size(); i++)
      {
         if (!contactStateB.contains(contactStateA.get(i)))
            return false;
      }
      return true;
   }

   private NewQuadrupedContactPhase createNewContactPhase(double startTime, List<RobotQuadrant> feetInContact, QuadrantDependentList<FramePoint3D> solePositions)
   {
      if (maxCapacity - size() > 0)
      {
         NewQuadrupedContactPhase contactPhase = add();
         contactPhase.getTimeInterval().setStartTime(startTime);
         contactPhase.setFeetInContact(feetInContact);
         contactPhase.setSolePosition(solePositions);

         contactPhase.update();

         return contactPhase;
      }
      else
      {
         return null;
      }
   }

}
