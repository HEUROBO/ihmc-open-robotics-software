package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class StraightLinePoseTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrame.constructFrameWithUnchangingTransformToParent("frameA", worldFrame,
         EuclidCoreRandomTools.nextRigidBodyTransform(random));

   private static final double EPSILON = 1.0e-10;

   @Test
   public void testCompareWithSingleFrameTrajectoryGenerators()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider initialPositionProvider = new PositionProvider()
      {
         @Override
         public void getPosition(FixedFramePoint3DBasics positionToPack)
         {
            positionToPack.set(initialPosition);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return worldFrame;
         }
      };
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider finalPositionProvider = new PositionProvider()
      {
         @Override
         public void getPosition(FixedFramePoint3DBasics positionToPack)
         {
            positionToPack.set(finalPosition);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return worldFrame;
         }
      };

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider initialOrientationProvider = orientationToPack -> orientationToPack.set(initialOrientation);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider finalOrientationProvider = orientationToPack -> orientationToPack.set(finalOrientation);

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FramePoint3D position1 = new FramePoint3D();
      FrameVector3D velocity1 = new FrameVector3D();
      FrameVector3D acceleration1 = new FrameVector3D();
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalPosition.compute(t);
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalPosition.getLinearData(position1, velocity1, acceleration1);
         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }
   }

   @Test
   public void testNegativeTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(-5.0);

      FramePoint3D position1 = new FramePoint3D(initialPosition);
      FrameVector3D velocity1 = new FrameVector3D(worldFrame);
      FrameVector3D acceleration1 = new FrameVector3D(worldFrame);
      FrameQuaternion orientation1 = new FrameQuaternion(initialOrientation);
      FrameVector3D angularVelocity1 = new FrameVector3D(worldFrame);
      FrameVector3D angularAcceleration1 = new FrameVector3D(worldFrame);

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

   @Test
   public void testTooBigTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(15.0);

      FramePoint3D position1 = new FramePoint3D(finalPosition);
      FrameVector3D velocity1 = new FrameVector3D(worldFrame);
      FrameVector3D acceleration1 = new FrameVector3D(worldFrame);
      FrameQuaternion orientation1 = new FrameQuaternion(finalOrientation);
      FrameVector3D angularVelocity1 = new FrameVector3D(worldFrame);
      FrameVector3D angularAcceleration1 = new FrameVector3D(worldFrame);

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

   @Test
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      final FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider initialPositionProvider = new PositionProvider()
      {
         @Override
         public void getPosition(FixedFramePoint3DBasics positionToPack)
         {
            positionToPack.set(initialPosition);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return worldFrame;
         }
      };
      final FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider finalPositionProvider = new PositionProvider()
      {
         @Override
         public void getPosition(FixedFramePoint3DBasics positionToPack)
         {
            positionToPack.set(finalPosition);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return worldFrame;
         }
      };

      final FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider initialOrientationProvider = orientationToPack -> orientationToPack.set(initialOrientation);
      final FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider finalOrientationProvider = orientationToPack -> orientationToPack.set(finalOrientation);

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position1", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation1", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FramePoint3D position1 = new FramePoint3D();
      FrameVector3D velocity1 = new FrameVector3D();
      FrameVector3D acceleration1 = new FrameVector3D();
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalPosition.compute(t);
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalPosition.getLinearData(position1, velocity1, acceleration1);
         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }

      // Do the same in another frame
      initialPosition.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, frameA, 100.0, 100.0, 100.0));
      initialPositionProvider = new PositionProvider()
      {
         @Override
         public void getPosition(FixedFramePoint3DBasics positionToPack)
         {
            positionToPack.set(initialPosition);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return frameA;
         }
      };
      finalPosition.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, frameA, 100.0, 100.0, 100.0));
      finalPositionProvider = new PositionProvider()
      {
         @Override
         public void getPosition(FixedFramePoint3DBasics positionToPack)
         {
            positionToPack.set(finalPosition);
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return frameA;
         }
      };

      initialOrientation.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, frameA));
      initialOrientationProvider = initialOrientationToPack -> initialOrientationToPack.set(initialOrientation);
      finalOrientation.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, frameA));
      finalOrientationProvider = finalOrientationToPack -> finalOrientationToPack.set(finalOrientation);

      originalPosition = new StraightLinePositionTrajectoryGenerator("position2", frameA, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation2", frameA, trajectoryTimeProvider, initialOrientationProvider,
            finalOrientationProvider, registry);

      trajToTest.switchTrajectoryFrame(frameA);
      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      position1.setToZero(frameA);
      velocity1.setToZero(frameA);
      acceleration1.setToZero(frameA);

      position2.setToZero(frameA);
      velocity2.setToZero(frameA);
      acceleration2.setToZero(frameA);

      orientation1.setToZero(frameA);
      angularVelocity1.setToZero(frameA);
      angularAcceleration1.setToZero(frameA);

      orientation2.setToZero(frameA);
      angularVelocity2.setToZero(frameA);
      angularAcceleration2.setToZero(frameA);

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalPosition.compute(t);
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalPosition.getLinearData(position1, velocity1, acceleration1);
         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }
   }
}
