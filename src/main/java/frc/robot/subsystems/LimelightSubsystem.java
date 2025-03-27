package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;

public class LimelightSubsystem extends SubsystemBase {
    private final String limelight;
    private final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

    // Existing dashboard entries for vision and target data
    private final GenericEntry dsCamera;
    private final GenericEntry dsPipeline;
    private final GenericEntry dsTx;
    private final GenericEntry dsTy;
    private final GenericEntry dsConfidence;
    private final GenericEntry dsTargetPoseRobotX;
    private final GenericEntry dsTargetPoseRobotY;
    private final GenericEntry dsTargetPoseRobotZ;
    private final GenericEntry dsTargetPoseRobotYaw;
    private final GenericEntry dsTargetPoseRobotPitch;
    private final GenericEntry dsTargetPoseRobotRoll;
    private final GenericEntry dsVisionPose;

    // New dashboard entries for separated pose estimate components
    private final GenericEntry dsPoseX;
    private final GenericEntry dsPoseY;
    private final GenericEntry dsPoseTheta;
    private final GenericEntry dsPoseTimestamp;
    private final GenericEntry dsPoseLatency;
    private final GenericEntry dsTagCount;
    private final GenericEntry dsTagSpan;
    private final GenericEntry dsTagAvgDist;
    private final GenericEntry dsTagAvgArea;

    public LimelightSubsystem(String llName) {
        this.limelight = llName;
        // Add basic limelight and vision values to Shuffleboard
        dsCamera = limelightTab.add(limelight, limelight).getEntry();
        dsCamera.setString(limelight);
        dsPipeline = limelightTab.add(limelight + "Pipeline", 0).getEntry();
        dsTx = limelightTab.add(limelight + "Tx", 0).getEntry();
        dsTy = limelightTab.add(limelight + "Ty", 0).getEntry();
        dsConfidence = limelightTab.add(limelight + "Confidence", 0).getEntry();
        dsTargetPoseRobotX = limelightTab.add(limelight + "TargetPoseRobotX", 0).getEntry();
        dsTargetPoseRobotY = limelightTab.add(limelight + "TargetPoseRobotY", 0).getEntry();
        dsTargetPoseRobotZ = limelightTab.add(limelight + "TargetPoseRobotZ", 0).getEntry();
        dsTargetPoseRobotYaw =
                limelightTab.add(limelight + "TargetPoseRobotYaw", 0).getEntry();
        dsTargetPoseRobotPitch =
                limelightTab.add(limelight + "TargetPoseRobotPitch", 0).getEntry();
        dsTargetPoseRobotRoll =
                limelightTab.add(limelight + "TargetPoseRobotRoll", 0).getEntry();
        dsVisionPose = limelightTab.add(limelight + "VisionPose", "No Data").getEntry();

        // Create new Shuffleboard entries for the separated pose estimate components.
        dsPoseX = limelightTab.add(limelight + "PoseX", 0.0).getEntry();
        dsPoseY = limelightTab.add(limelight + "PoseY", 0.0).getEntry();
        dsPoseTheta = limelightTab.add(limelight + "PoseTheta", 0.0).getEntry();
        dsPoseTimestamp = limelightTab.add(limelight + "PoseTimestamp", 0.0).getEntry();
        dsPoseLatency = limelightTab.add(limelight + "PoseLatency", 0.0).getEntry();
        dsTagCount = limelightTab.add(limelight + "TagCount", 0).getEntry();
        dsTagSpan = limelightTab.add(limelight + "TagSpan", 0.0).getEntry();
        dsTagAvgDist = limelightTab.add(limelight + "TagAvgDist", 0.0).getEntry();
        dsTagAvgArea = limelightTab.add(limelight + "TagAvgArea", 0.0).getEntry();
    }

    /** Retrieves the latest vision pose estimate. Returns a default pose if none is available. */
    public LimelightHelpers.PoseEstimate getVisionPose() {
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        return (poseEstimate != null) ? poseEstimate : new LimelightHelpers.PoseEstimate();
    }

    /**
     * Sets the camera's pose relative to the robot. Note: The helper method expects rotation in the order: roll, pitch,
     * then yaw.
     */
    public void setCameraPose_RobotSpace(double x, double y, double z, double yaw, double pitch, double roll) {
        LimelightHelpers.setCameraPose_RobotSpace(limelight, x, y, z, roll, pitch, yaw);
    }

    /** Checks if a valid target is detected. */
    public boolean getTV() {
        return LimelightHelpers.getTV(limelight);
    }

    /** Returns the horizontal offset (Tx) if a target is visible. */
    public double getTx() {
        return getTV() ? LimelightHelpers.getTX(limelight) : 0.0;
    }

    /** Returns the vertical offset (Ty) if a target is visible. */
    public double getTy() {
        return getTV() ? LimelightHelpers.getTY(limelight) : 0.0;
    }

    /** Sets the active pipeline index. */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(limelight, pipeline);
    }

    /** Sets the stream mode to Picture-in-Picture with the main stream. */
    public void setMainPIP() {
        LimelightHelpers.setStreamMode_PiPMain(limelight);
    }

    /** Sets the stream mode to Picture-in-Picture with the secondary stream. */
    public void setSecondaryPIP() {
        LimelightHelpers.setStreamMode_PiPSecondary(limelight);
    }

    /** Calculates a confidence value based on target area and latency. */
    public double getConfidence() {
        double ta = LimelightHelpers.getTA(limelight);
        boolean tv = getTV();
        double latency = getVisionPose().latency;
        if (!tv) {
            return 0.0;
        }
        double confidence = Math.min(ta / 1.0, 1.0);
        if (latency > 50) {
            confidence *= 0.5;
        }
        return confidence;
    }

    /** Retrieves the complete set of Limelight results. */
    public LimelightResults getLimelightResults() {
        return LimelightHelpers.getLatestResults(limelight);
    }

    /** Retrieves the target's pose relative to the robot. */
    public double[] getTargetInRobotSpace() {
        return LimelightHelpers.getTargetPose_RobotSpace(limelight);
    }

    // Helper method to cache the target pose for the current periodic update
    private double[] getCachedTargetPose() {
        return getTargetInRobotSpace();
    }

    // These methods extract individual components from the target pose array.
    public double getTagToRobotX() {
        double[] targetPose = getCachedTargetPose();
        return (targetPose.length > 0) ? targetPose[0] : 0.0;
    }

    public double getTagToRobotY() {
        double[] targetPose = getCachedTargetPose();
        return (targetPose.length > 1) ? targetPose[1] : 0.0;
    }

    public double getTagToRobotZ() {
        double[] targetPose = getCachedTargetPose();
        return (targetPose.length > 2) ? targetPose[2] : 0.0;
    }

    public double getTagToRobotYaw() {
        double[] targetPose = getCachedTargetPose();
        return (targetPose.length > 3) ? targetPose[3] : 0.0;
    }

    public double getTagToRobotPitch() {
        double[] targetPose = getCachedTargetPose();
        return (targetPose.length > 4) ? targetPose[4] : 0.0;
    }

    public double getTagToRobotRoll() {
        double[] targetPose = getCachedTargetPose();
        return (targetPose.length > 5) ? targetPose[5] : 0.0;
    }

    @Override
    public void periodic() {
        // Update target pose values from the NetworkTables
        double[] targetPose = getTargetInRobotSpace();

        dsTx.setDouble(getTx());
        dsTy.setDouble(getTy());
        dsConfidence.setDouble(getConfidence());

        if (targetPose.length >= 6) {
            dsTargetPoseRobotX.setDouble(targetPose[0]);
            dsTargetPoseRobotY.setDouble(targetPose[1]);
            dsTargetPoseRobotZ.setDouble(targetPose[2]);
            dsTargetPoseRobotYaw.setDouble(targetPose[3]);
            dsTargetPoseRobotPitch.setDouble(targetPose[4]);
            dsTargetPoseRobotRoll.setDouble(targetPose[5]);
        } else {
            dsTargetPoseRobotX.setDouble(0);
            dsTargetPoseRobotY.setDouble(0);
            dsTargetPoseRobotZ.setDouble(0);
            dsTargetPoseRobotYaw.setDouble(0);
            dsTargetPoseRobotPitch.setDouble(0);
            dsTargetPoseRobotRoll.setDouble(0);
        }

        // Retrieve the complete vision pose estimate.
        LimelightHelpers.PoseEstimate visionPose = getVisionPose();

        // Check if there are any tags detected
        if (visionPose.tagCount > 0) {
            dsVisionPose.setString(visionPose.toString());
            // Separate the pose estimate into its components.
            Pose2d pose = visionPose.pose;
            dsPoseX.setDouble(pose.getTranslation().getX());
            dsPoseY.setDouble(pose.getTranslation().getY());
            dsPoseTheta.setDouble(pose.getRotation().getDegrees());

            dsPoseTimestamp.setDouble(visionPose.timestampSeconds);
            dsPoseLatency.setDouble(visionPose.latency);
            dsTagCount.setDouble(visionPose.tagCount);
            dsTagSpan.setDouble(visionPose.tagSpan);
            dsTagAvgDist.setDouble(visionPose.avgTagDist);
            dsTagAvgArea.setDouble(visionPose.avgTagArea);
        } else {
            // If no tags are detected, clear or reset the dashboard values.
            dsVisionPose.setString("No Tags");
            dsPoseX.setDouble(0.0);
            dsPoseY.setDouble(0.0);
            dsPoseTheta.setDouble(0.0);
            dsPoseTimestamp.setDouble(0.0);
            dsPoseLatency.setDouble(0.0);
            dsTagCount.setDouble(0);
            dsTagSpan.setDouble(0.0);
            dsTagAvgDist.setDouble(0.0);
            dsTagAvgArea.setDouble(0.0);
        }
    }
}
