package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveDrive;

import frc.robot.Constants;
import frc.robot.Constants.CameraConfig;

/**
 * Vision subsystem using PhotonVision that plugs into swervelib's pose estimator.
 */
public class Vision extends SubsystemBase {

  // ====== User-tunable constants ======
  /** Must match the name set in the PhotonVision UI. */
  public static final String kCameraName = "photonvision";

  /** Std-devs when we only trust a single tag (meters for x/y, radians for heading). */
  private static final Matrix<N3, N1> kSingleTagStdDevs =
      VecBuilder.fill(0.9, 0.9, Math.toRadians(25.0));

  /** Std-devs when multiple tags contribute to the estimate (tighter). */
  private static final Matrix<N3, N1> kMultiTagStdDevs =
      VecBuilder.fill(0.3, 0.3, Math.toRadians(7.0));

  // ====== Context from the drive (for telemetry only) ======
  private final Supplier<Pose2d> currentPoseSupplier; // used in sendable
  private final Field2d field; // for visualization

  // One entry per camera.
  private final Map<String, Cam> cams = new LinkedHashMap<>();

  public Vision(Supplier<Pose2d> currentPoseSupplier, Field2d field) {
    this.currentPoseSupplier = currentPoseSupplier;
    this.field = field;

    // Build a camera + estimator for each configured camera in Constants.Vision.CAMERAS
    for (CameraConfig cfg : Constants.Vision.CAMERAS) {
      PhotonCamera camera = new PhotonCamera(cfg.name);

      PhotonPoseEstimator estimator = new PhotonPoseEstimator(
          Constants.APRIL_TAG_FIELD_LAYOUT,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          cfg.robotToCam
      );

      // Reasonable fallback if multi-tag not available
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cams.put(cfg.name, new Cam(camera, estimator));
    }
  }

  @Override
  public void periodic() {
    Pose2d refPose = currentPoseSupplier.get();

    for (var entry : cams.entrySet()) {
      String name = entry.getKey();
      Cam cam = entry.getValue();

      // Get latest frame and seed the estimator with odometry as a reference.
      cam.lastResult = cam.camera.getLatestResult();
      cam.estimator.setReferencePose(refPose);

      // Estimate robot pose from this camera's frame.
      cam.lastEstimate = cam.estimator.update(cam.lastResult);

      // Minimal per-camera visualization: estimate + seen tags
      if (field != null) {
        if (cam.lastEstimate.isPresent()) {
          field.getObject("Vision/" + name + "/Estimate")
               .setPose(cam.lastEstimate.get().estimatedPose.toPose2d());
        }

        var seen = new ArrayList<Pose2d>();
        if (cam.lastResult.hasTargets()) {
          for (PhotonTrackedTarget t : cam.lastResult.getTargets()) {
            try {
              Optional<Pose3d> tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(t.getFiducialId());
              tagPose.ifPresent(p3 -> seen.add(p3.toPose2d()));
            } catch (Exception ignored) {}
          }
        }
        field.getObject("Vision/" + name + "/SeenTags").setPoses(seen);
      }
    }
  }

  /**
   * Pushes all valid camera estimates into the pose estimator.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    for (Cam cam : cams.values()) {
      if (cam.lastEstimate.isEmpty()) continue;

      EstimatedRobotPose est = cam.lastEstimate.get();
      Matrix<N3, N1> stdDevs =
          (est.targetsUsed.size() >= 2) ? kMultiTagStdDevs : kSingleTagStdDevs;

      double ts = est.timestampSeconds;

      // Add each camera's measurement independently. The filter weighs them via stdDevs and time.
      swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), ts, stdDevs);
    }
  }

  // =================== Public helpers (optional) ===================

  /** Access a camera's latest PhotonVision frame by name. */
  public Optional<PhotonPipelineResult> getLastResult(String cameraName) {
    Cam cam = cams.get(cameraName);
    return (cam == null) ? Optional.empty() : Optional.ofNullable(cam.lastResult);
    }

  /** Access a camera's latest pose estimate by name. */
  public Optional<EstimatedRobotPose> getLatestEstimate(String cameraName) {
    Cam cam = cams.get(cameraName);
    return (cam == null) ? Optional.empty() : cam.lastEstimate;
  }

  /** Utility: compute a field-relative pose from a tag + 2D offset. */
  public static Pose2d poseFromTag(int tagId, Transform2d robotOffset) {
    try {
      Optional<Pose3d> tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
      return tagPose.map(p -> p.toPose2d().transformBy(robotOffset)).orElseGet(Pose2d::new);
    } catch (Exception e) {
      return new Pose2d();
    }
  }

  // =================== Internal per-camera struct ===================

  private static class Cam {
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;

    PhotonPipelineResult lastResult = new PhotonPipelineResult();
    Optional<EstimatedRobotPose> lastEstimate = Optional.empty();

    Cam(PhotonCamera camera, PhotonPoseEstimator estimator) {
      this.camera = camera;
      this.estimator = estimator;
    }
  }

}
