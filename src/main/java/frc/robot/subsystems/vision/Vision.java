package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.telemetry.ElasticTelemetry;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;

  private final AprilTagFieldLayout m_fieldLayout;

  private final PhotonCamera m_camera1;
  private final PhotonCamera m_camera2;
  private final PhotonCamera m_camera3;

  private final PhotonPoseEstimator m_estimator1;
  private final PhotonPoseEstimator m_estimator2;
  private final PhotonPoseEstimator m_estimator3;

  public Vision(DriveSubsystem driveSubsystem, AprilTagFieldLayout fieldLayout) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_fieldLayout = fieldLayout;

    m_camera1 = new PhotonCamera(VisionConstants.CAMERA_1_NAME);
    m_camera2 = new PhotonCamera(VisionConstants.CAMERA_2_NAME);
    m_camera3 = new PhotonCamera(VisionConstants.CAMERA_3_NAME);

    if (m_fieldLayout != null) {
      m_estimator1 = new PhotonPoseEstimator(m_fieldLayout, VisionConstants.ROBOT_TO_CAMERA_1);
      m_estimator2 = new PhotonPoseEstimator(m_fieldLayout, VisionConstants.ROBOT_TO_CAMERA_2);
      m_estimator3 = new PhotonPoseEstimator(m_fieldLayout, VisionConstants.ROBOT_TO_CAMERA_3);
    } else {
      m_estimator1 = null;
      m_estimator2 = null;
      m_estimator3 = null;
    }
  }

  @Override
  public void periodic() {
    double pitch = Math.abs(m_driveSubsystem.getPitch());
    double roll = Math.abs(m_driveSubsystem.getRoll());
    boolean rejectForTilt =
        !DriverStation.isDisabled()
            && (pitch > VisionConstants.MAX_PITCH_ROLL_DEGREES
                || roll > VisionConstants.MAX_PITCH_ROLL_DEGREES);
    ElasticTelemetry.setBoolean("Vision/IgnoringDueToPitchRoll", rejectForTilt);

    processEstimator(m_estimator1, m_camera1, pitch, roll, rejectForTilt);
    processEstimator(m_estimator2, m_camera2, pitch, roll, rejectForTilt);
    processEstimator(m_estimator3, m_camera3, pitch, roll, rejectForTilt);
  }

  private void processEstimator(
      PhotonPoseEstimator estimator,
      PhotonCamera camera,
      double pitch,
      double roll,
      boolean rejectForTilt) {
    String prefix = "Vision/" + camera.getName() + "/";
    ElasticTelemetry.setNumber(prefix + "Gyro/Pitch", pitch);
    ElasticTelemetry.setNumber(prefix + "Gyro/Roll", roll);

    if (estimator == null) {
      publishNoResultTelemetry(prefix);
      ElasticTelemetry.setString(prefix + "Status", "No estimator");
      return;
    }
    if (!camera.isConnected()) {
      publishNoResultTelemetry(prefix);
      ElasticTelemetry.setString(prefix + "Status", "Not connected");
      return;
    }

    var results = camera.getAllUnreadResults();
    ElasticTelemetry.setNumber(prefix + "ResultCount", results.size());
    if (results.isEmpty()) {
      publishNoResultTelemetry(prefix);
      ElasticTelemetry.setString(prefix + "Status", "No results");
      return;
    }

    // Skip pitch/roll check when disabled so vision can establish starting pose
    if (rejectForTilt) {
      ElasticTelemetry.setBoolean(prefix + "HasTargets", false);
      ElasticTelemetry.setNumber(prefix + "TargetCount", 0);
      ElasticTelemetry.setBoolean(prefix + "CoprocMultiTag", false);
      ElasticTelemetry.setBoolean(prefix + "LowestAmbiguity", false);
      ElasticTelemetry.setString(prefix + "Status", "Rejected: pitch/roll");
      return;
    }

    for (var result : results) {
      ElasticTelemetry.setNumber(
          prefix + "LatencyMs", (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0);
      ElasticTelemetry.setBoolean(prefix + "HasTargets", result.hasTargets());
      ElasticTelemetry.setNumber(prefix + "TargetCount", result.getTargets().size());
      ElasticTelemetry.setBoolean(prefix + "LowestAmbiguity", false);

      // Try coprocessor multi-tag first (real robot), fall back to lowest ambiguity (sim/single
      // tag)
      var estimatedPose = estimator.estimateCoprocMultiTagPose(result);
      ElasticTelemetry.setBoolean(prefix + "CoprocMultiTag", estimatedPose.isPresent());

      if (estimatedPose.isEmpty()) {
        // Reject high-ambiguity single-tag estimates (noisy / flipped pose risk)
        var bestTarget = result.hasTargets() ? result.getBestTarget() : null;
        if (bestTarget != null) {
          double ambiguity = bestTarget.getPoseAmbiguity();
          ElasticTelemetry.setNumber(prefix + "Ambiguity", ambiguity);
          if (ambiguity > VisionConstants.MAX_AMBIGUITY) {
            ElasticTelemetry.setString(prefix + "Status", "Rejected: ambiguity " + ambiguity);
            continue;
          }
        }
        estimatedPose = estimator.estimateLowestAmbiguityPose(result);
        ElasticTelemetry.setBoolean(prefix + "LowestAmbiguity", estimatedPose.isPresent());
      }

      if (estimatedPose.isPresent()) {
        var est = estimatedPose.get();
        Pose2d pose2d = est.estimatedPose.toPose2d();
        ElasticTelemetry.setString(prefix + "EstPose", pose2d.toString());

        if (isVisionPoseValid(pose2d, m_driveSubsystem.getPose())) {
          double translationResidual =
              pose2d.getTranslation().getDistance(m_driveSubsystem.getPose().getTranslation());
          double headingResidualDeg =
              pose2d.getRotation().minus(m_driveSubsystem.getPose().getRotation()).getDegrees();
          ElasticTelemetry.setNumber(prefix + "Residual/TranslationMeters", translationResidual);
          ElasticTelemetry.setNumber(prefix + "Residual/HeadingDegrees", headingResidualDeg);
          ElasticTelemetry.setNumber(
              prefix + "Residual/PoseAgeMs",
              (Timer.getFPGATimestamp() - est.timestampSeconds) * 1000.0);
          m_driveSubsystem.addVisionMeasurement(pose2d, est.timestampSeconds);
          m_driveSubsystem.getField().getObject("VisionPose_" + camera.getName()).setPose(pose2d);
          ElasticTelemetry.setString(prefix + "Status", "Applied");
        } else {
          ElasticTelemetry.setString(prefix + "Status", "Rejected: validation");
        }
      } else {
        ElasticTelemetry.setString(prefix + "Status", "No pose estimate");
      }
    }
  }

  private void publishNoResultTelemetry(String prefix) {
    ElasticTelemetry.setNumber(prefix + "ResultCount", 0);
    ElasticTelemetry.setBoolean(prefix + "HasTargets", false);
    ElasticTelemetry.setNumber(prefix + "TargetCount", 0);
    ElasticTelemetry.setBoolean(prefix + "CoprocMultiTag", false);
    ElasticTelemetry.setBoolean(prefix + "LowestAmbiguity", false);
  }

  /**
   * Basic results validator: 1. Must be inside the field bounds (with a 0.5m buffer) 2. Prevent
   * "light speed" teleportation by rejecting poses too far from current odometry
   *
   * @param visionPose The newly estimated 2D pose from PhotonVision
   * @param currentPose The 2D pose from the DriveSubsystem odometry
   * @return true if the pose passes all sanity checks
   */
  private boolean isVisionPoseValid(Pose2d visionPose, Pose2d currentPose) {
    boolean insideField =
        visionPose.getX() >= -0.5
            && visionPose.getX() <= VisionConstants.FIELD_LENGTH_METERS + 0.5
            && visionPose.getY() >= -0.5
            && visionPose.getY() <= VisionConstants.FIELD_WIDTH_METERS + 0.5;

    // Skip the jump check when disabled — allows vision to establish the correct
    // starting position when the robot is first placed on the field.
    boolean jumpTooLarge =
        !DriverStation.isDisabled()
            && visionPose.getTranslation().getDistance(currentPose.getTranslation())
                > VisionConstants.MAX_POSE_DIFFERENCE_METERS;

    if (!insideField || jumpTooLarge) {
      ElasticTelemetry.setString(
          "Vision/RejectedPoseReason", !insideField ? "Outside Field" : "Jump Too Large");
      return false;
    }

    return true;
  }

  public PhotonCamera getCamera1() {
    return m_camera1;
  }

  public PhotonCamera getCamera2() {
    return m_camera2;
  }

  public PhotonCamera getCamera3() {
    return m_camera3;
  }
}
