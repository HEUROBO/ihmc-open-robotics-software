package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.EjmlUnitTests;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.stateEstimation.CenterOfMassBasedFullRobotModelUpdater;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.NullControlFlowElement;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LinearAccelerationMeasurementModelElementTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   // TODO: test internal accelerations
   @Test
   public void test()
   {
      Random random = new Random(125125523L);
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z};
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);
      RigidBody elevator = randomFloatingChain.getElevator();
      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();

      RigidBody estimationLink = randomFloatingChain.getRootJoint().getSuccessor();
      ReferenceFrame estimationFrame = randomFloatingChain.getRootJoint().getFrameAfterJoint();
      RigidBody measurementLink = estimationLink;    // randomFloatingChain.getRevoluteJoints().get(jointAxes.length - 1).getSuccessor();
      ReferenceFrame measurementFrame = measurementLink.getBodyFixedFrame();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, false);

      String name = "test";
      YoVariableRegistry registry = new YoVariableRegistry(name);

      ControlFlowOutputPort<FramePoint> centerOfMassPositionPort = new ControlFlowOutputPort<FramePoint>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);

      ControlFlowOutputPort<FrameOrientation> orientationPort = new ControlFlowOutputPort<FrameOrientation>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularVelocityPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularAccelerationPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);

      ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort = new ControlFlowInputPort<Vector3d>(controlFlowElement);
      double gZ = 9.81;

      LinearAccelerationMeasurementModelElement modelElement = new LinearAccelerationMeasurementModelElement(name, registry, centerOfMassPositionPort,
                                                                  centerOfMassVelocityPort, centerOfMassAccelerationPort, orientationPort, angularVelocityPort,
                                                                  angularAccelerationPort, linearAccelerationMeasurementInputPort, twistCalculator,
                                                                  spatialAccelerationCalculator, measurementLink, measurementFrame, estimationLink,
                                                                  estimationFrame, gZ);

      randomFloatingChain.setRandomPositionsAndVelocities(random);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      Runnable updater = new CenterOfMassBasedFullRobotModelUpdater(twistCalculator, spatialAccelerationCalculator, centerOfMassPositionPort,
                            centerOfMassVelocityPort, centerOfMassAccelerationPort, orientationPort, angularVelocityPort, angularAccelerationPort,
                            estimationLink, estimationFrame, rootJoint);

      centerOfMassPositionPort.setData(new FramePoint(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
      centerOfMassVelocityPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
      centerOfMassAccelerationPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
      Matrix3d orientation = new Matrix3d();
      orientation.set(RandomTools.generateRandomRotation(random));
      orientationPort.setData(new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation));
      angularVelocityPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));
      angularAccelerationPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));

      updater.run();
      spatialAccelerationCalculator.compute();
      setMeasuredLinearAccelerationToActual(spatialAccelerationCalculator, measurementLink, measurementFrame, linearAccelerationMeasurementInputPort);
      setCenterOfMassAccelerationToActual(elevator, centerOfMassAccelerationPort);
      setAngularAccelerationToActual(spatialAccelerationCalculator, estimationLink, estimationFrame, angularAccelerationPort);

      DenseMatrix64F zeroResidual = modelElement.computeResidual();
      DenseMatrix64F zeroVector = new DenseMatrix64F(3, 1);
      EjmlUnitTests.assertEquals(zeroVector, zeroResidual, 1e-12);

      double perturbation = 1e-6;
      double tol = 1e-12;
      modelElement.computeMatrixBlocks();

      // CoM acceleration perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(centerOfMassAccelerationPort, modelElement,
              new FrameVector(centerOfMassAccelerationPort.getData()), perturbation, tol, updater);

      // CoM velocity perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(centerOfMassVelocityPort, modelElement,
              new FrameVector(centerOfMassVelocityPort.getData()), perturbation, tol, updater);

      // orientation perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(orientationPort, modelElement, new FrameOrientation(orientationPort.getData()),
              perturbation, tol, updater);

      // angular velocity perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(angularVelocityPort, modelElement, new FrameVector(angularVelocityPort.getData()),
              perturbation, tol, updater);

      // angular acceleration perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(angularAccelerationPort, modelElement,
              new FrameVector(angularAccelerationPort.getData()), perturbation, tol, updater);

   }

   private static void setCenterOfMassAccelerationToActual(RigidBody elevator, ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort)
   {
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, rootAcceleration, false);
      FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      centerOfMassAccelerationCalculator.packCoMAcceleration(comAcceleration);
      centerOfMassAccelerationPort.setData(comAcceleration);
   }

   private static void setMeasuredLinearAccelerationToActual(SpatialAccelerationCalculator spatialAccelerationCalculator, RigidBody measurementLink,
           ReferenceFrame measurementFrame, ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort)
   {
      FramePoint measurementPoint = new FramePoint(measurementFrame);
      FrameVector linearAcceleration = new FrameVector(measurementFrame);
      spatialAccelerationCalculator.packLinearAccelerationOfBodyFixedPoint(linearAcceleration, measurementLink, measurementPoint);
      linearAcceleration.changeFrame(measurementFrame);
      linearAccelerationMeasurementInputPort.setData(linearAcceleration.getVectorCopy());
   }

   private static void setAngularAccelerationToActual(SpatialAccelerationCalculator spatialAccelerationCalculator, RigidBody estimationLink,
           ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameVector> angularAccelerationPort)
   {
      SpatialAccelerationVector estimationLinkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packAccelerationOfBody(estimationLinkAcceleration, estimationLink);
      FrameVector angularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      estimationLinkAcceleration.packAngularPart(angularAcceleration);
      angularAcceleration.changeFrame(estimationFrame);
      angularAccelerationPort.setData(angularAcceleration);
   }
}
