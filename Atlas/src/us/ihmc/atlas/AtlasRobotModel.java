package us.ihmc.atlas;

import java.net.URI;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasArmControllerParameters;
import us.ihmc.atlas.parameters.AtlasCapturePointPlannerParameters;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasDefaultArmConfigurations;
import us.ihmc.atlas.parameters.AtlasDrivingControllerParameters;
import us.ihmc.atlas.parameters.AtlasFootstepParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasRobotMultiContactControllerParameters;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.parameters.AtlasStateEstimatorParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.atlas.physics.AtlasPhysicsEngineConfiguration;
import us.ihmc.atlas.ros.AtlasPPSTimestampOffsetProvider;
import us.ihmc.atlas.sensors.AtlasSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.iRobot.control.IRobotHandCommandManager;
import us.ihmc.ihmcPerception.footstepPlanner.FootstepParameters;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotiq.control.RobotiqHandCommandManager;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.robotiq.simulatedHand.SimulatedRobotiqHandsController;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

import com.jme3.math.Transform;

public class AtlasRobotModel implements DRCRobotModel
{
   public enum AtlasTarget
   {
      SIM, GAZEBO, REAL_ROBOT
   }

   private final AtlasRobotVersion selectedVersion;
   private final AtlasTarget target;

   private static final long ESTIMATOR_DT_IN_NS = 1000000;
   private static final double ESTIMATOR_DT = TimeTools.nanoSecondstoSeconds(ESTIMATOR_DT_IN_NS);
   private static final double CONTROL_DT = 0.006;

   private static final double ATLAS_ONBOARD_SAMPLINGFREQ = 1000.0;
   public static final double ATLAS_ONBOARD_DT = 1.0 / ATLAS_ONBOARD_SAMPLINGFREQ;

   private final JaxbSDFLoader loader;

   private final AtlasJointMap jointMap;
   private final AtlasSensorInformation sensorInformation;
   private final AtlasArmControllerParameters armControllerParameters;
   private final AtlasCapturePointPlannerParameters capturePointPlannerParameters;
   private final AtlasWalkingControllerParameters walkingControllerParameters;
   private final AtlasStateEstimatorParameters stateEstimatorParameters;
   private final AtlasRobotMultiContactControllerParameters multiContactControllerParameters;
   private final AtlasDrivingControllerParameters drivingControllerParameters;
   private final AtlasDefaultArmConfigurations defaultArmConfigurations;

   @Override
   public WholeBodyIkSolver createWholeBodyIkSolver()
   {
      return new AtlasWholeBodyIK(this);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, AtlasTarget target, boolean headless)
   {
      selectedVersion = atlasVersion;
      jointMap = new AtlasJointMap(selectedVersion);
      this.target = target;

      if (!headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(selectedVersion.getResourceDirectories(), selectedVersion.getSdfFileAsStream(), false);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[] {}, selectedVersion.getSdfFileAsStream(), headless);
      }

      for (String forceSensorNames : AtlasSensorInformation.forceSensorNames)
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new RigidBodyTransform());
      }

      boolean runningOnRealRobot = target == AtlasTarget.REAL_ROBOT;
      capturePointPlannerParameters = new AtlasCapturePointPlannerParameters(runningOnRealRobot);
      sensorInformation = new AtlasSensorInformation(target);
      armControllerParameters = new AtlasArmControllerParameters(runningOnRealRobot);
      walkingControllerParameters = new AtlasWalkingControllerParameters(target, jointMap);
      stateEstimatorParameters = new AtlasStateEstimatorParameters(jointMap, runningOnRealRobot, getEstimatorDT());
      multiContactControllerParameters = new AtlasRobotMultiContactControllerParameters(jointMap);
      drivingControllerParameters = new AtlasDrivingControllerParameters(jointMap);
      defaultArmConfigurations = new AtlasDefaultArmConfigurations();
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return armControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParameters;
   }

   @Override
   public AtlasPhysicalProperties getPhysicalProperties()
   {
      return new AtlasPhysicalProperties();
   }

   @Override
   public AtlasJointMap getJointMap()
   {
      return jointMap;
   }

   public DRCHandType getHandType()
   {
      return selectedVersion.getHandModel();
   }

   public AtlasRobotVersion getAtlasVersion()
   {
      return selectedVersion;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      return selectedVersion.getOffsetFromAttachmentPlate(side);
   }

   @Override
   public RigidBodyTransform getTransform3dWristToHand(RobotSide side)
   {
      return JMEGeometryUtils.transformFromJMECoordinatesToZup(getJmeTransformWristToHand(side));
   }

   @Override
   public String toString()
   {
      return selectedVersion.toString();
   }

   @Override
   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new AtlasSimInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return multiContactControllerParameters;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot sdfRobot)
   {
      return new AtlasPhysicsEngineConfiguration(getJointMap(), sdfRobot);
   }

   @Override
   public AtlasContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
   }

   public void createHandContactPoints()
   {
      jointMap.getContactPointParameters().createHandContactPoints();
   }

   public void addMoreFootContactPointsSimOnly()
   {
      jointMap.getContactPointParameters().addMoreFootContactPointsSimOnly();
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      AtlasDampingParameters.setDampingParameters(simulatedRobot, getHandType(), getJointMap());
   }

   @Override
   public HandModel getHandModel()
   {
      if (selectedVersion.hasRobotiqHands())
         return new RobotiqHandModel();
      return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return drivingControllerParameters;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public SDFFullRobotModel createFullRobotModel()
   {
      return loader.createFullRobotModel(getJointMap(), sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public SDFRobot createSdfRobot(boolean createCollisionMeshes)
   {
      return loader.createRobot(getJointMap(), createCollisionMeshes);
   }

   @Override
   public double getSimulateDT()
   {
      return 0.0001;
   }

   @Override
   public double getEstimatorDT()
   {
      return ESTIMATOR_DT;
   }

   @Override
   public double getControllerDT()
   {
      return CONTROL_DT;
   }

   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      if (target == AtlasTarget.REAL_ROBOT)
      {
         return new AtlasPPSTimestampOffsetProvider(sensorInformation);
      }

      if (target == AtlasTarget.SIM && DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
      {
         return new SimulationRosClockPPSTimestampOffsetProvider();
      }

      return new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager(URI rosCoreURI)
   {
      return new AtlasSensorSuiteManager(rosCoreURI, getPPSTimestampOffsetProvider(), sensorInformation, getJointMap(), getPhysicalProperties(),
            getFootstepParameters());
   }

   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public HandCommandManager createHandCommandManager(AbstractNetworkProcessorNetworkingManager networkManager)
   {
      if (target == AtlasTarget.REAL_ROBOT)
      {
         switch (getHandType())
         {
         case IROBOT:
            return new IRobotHandCommandManager(networkManager);

         case ROBOTIQ:
            return new RobotiqHandCommandManager(networkManager);

         default:
            break;

         }
      }

      return null;
   }

   @Override
   public DRCHandType getDRCHandType()
   {
      return selectedVersion.getHandModel();
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(SDFRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
         GlobalDataProducer globalDataProducer)
   {
      switch (getHandType())
      {
      case ROBOTIQ:
         return new SimulatedRobotiqHandsController(simulatedRobot, this, threadDataSynchronizer, globalDataProducer);
      default:
         return null;
      }
   }

   @Override
   public FootstepParameters getFootstepParameters()
   {
      // TODO Auto-generated method stub
      return new AtlasFootstepParameters();
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), selectedVersion.getSdfFileAsStream(), selectedVersion.getResourceDirectories());
   }

   @Override
   public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      return null;
   }

   @Override
   public LogSettings getLogSettings()
   {
      switch (target)
      {
      case REAL_ROBOT:
         return LogSettings.ATLAS_IAN;
      case GAZEBO:
      case SIM:
      default:
         return LogSettings.SIMULATION;
      }
   }

   @Override
   public DefaultArmConfigurations getDefaultArmConfigurations()
   {
      return defaultArmConfigurations;
   }

   @Override
   public Pair<Class<?>, String[]> getOperatorInterfaceStarter()
   {
      String[] args = { "-m " + getAtlasVersion().name() };
      return new Pair<Class<?>, String[]>(AtlasOperatorUserInterface.class, args);
   }
}
