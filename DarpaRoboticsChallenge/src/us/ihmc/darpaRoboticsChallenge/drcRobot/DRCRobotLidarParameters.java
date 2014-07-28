package us.ihmc.darpaRoboticsChallenge.drcRobot;

public class DRCRobotLidarParameters implements DRCRobotSensorParameters
{
   private final String lidarSensorName;
   private final String laserTopic;
   private final String lidarSpindleJointName;
   private final String lidarSpindleJointTopic;
   private final String lidarBaseFrameForRos;
   private final String lidarEndFrameForRos;
   private final String poseFrameName;
   private final double lidarSpinVelocity;
   private final boolean useRosForTransformFromPoseToSensor;
   private final int lidarId;

   public DRCRobotLidarParameters(String lidarSensorName, String laserTopic, String lidarSpindleJointName, String lidarSpindleJointTopic,
         String lidarToRosHandoffFrameInSdf, String lidarBaseFrameForRos, String lidarEndFrameForRos, double lidar_spindle_velocity, int lidarId)
   {
      this.lidarSensorName = lidarSensorName;//"head_hokuyo_sensor"
      this.laserTopic = laserTopic;//multisense_namespace+"/lidar_scan"
      this.lidarSpindleJointName = lidarSpindleJointName;//"hokuyo_joint";
      this.lidarSpindleJointTopic = lidarSpindleJointTopic;//multisense_namespace + "/joint_states";
      this.poseFrameName = lidarToRosHandoffFrameInSdf;//"head";
      this.lidarBaseFrameForRos = lidarBaseFrameForRos;//multisense_namespace + "/head";
      this.lidarEndFrameForRos = lidarEndFrameForRos;//multisense_namespace + "/head_hokuyo_frame";
      this.lidarSpinVelocity = lidar_spindle_velocity;//lidar_spindle_velocity = 5.1;
      this.useRosForTransformFromPoseToSensor = true;
      this.lidarId = lidarId;
   }
   
   public DRCRobotLidarParameters(String lidarSensorName, String laserTopic, String lidarSpindleJointName, String lidarSpindleJointTopic,
         String sdflinkNameForSensorPose, double lidar_spindle_velocity, int lidarId)
   {
      this.lidarSensorName = lidarSensorName;
      this.laserTopic = laserTopic;
      this.lidarSpindleJointName = lidarSpindleJointName;
      this.lidarSpindleJointTopic = lidarSpindleJointTopic;
      this.poseFrameName = sdflinkNameForSensorPose;
      this.lidarBaseFrameForRos = null;
      this.lidarEndFrameForRos = null;
      this.lidarSpinVelocity = lidar_spindle_velocity;
      this.useRosForTransformFromPoseToSensor = false;
      this.lidarId = lidarId;
   }

   public String getSensorNameInSdf()
   {
      return lidarSensorName;
   }

   public String getRosTopic()
   {
      return laserTopic;
   }

   public String getLidarSpindleJointName()
   {
      return lidarSpindleJointName;
   }

   public String getLidarSpindleJointTopic()
   {
      return lidarSpindleJointTopic;
   }

   public String getBaseFrameForRosTransform()
   {
      return lidarBaseFrameForRos;
   }

   public String getEndFrameForRosTransform()
   {
      return lidarEndFrameForRos;
   }

   public double getLidarSpindleVelocity()
   {
      return lidarSpinVelocity;
   }

   public int getSensorId()
   {
      return lidarId;
   }

   public boolean useRosForTransformFromPoseToSensor()
   {
      return useRosForTransformFromPoseToSensor;
   }

   @Override
   public String getPoseFrameForSdf()
   {
      return poseFrameName;
   }

   @Override
   public DRCRobotSensorType getSensorType()
   {
      return DRCRobotSensorType.LIDAR;
   }
}