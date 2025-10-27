package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CameraConfig;

/**
 * Vision subsystem using PhotonVision that plugs into swervelib's pose estimator.
 */
public class Vision extends SubsystemBase {

  // ====== Context from the drive (for telemetry only) ======
  private final Supplier<Pose2d> currentPoseSupplier; // used in sendable
  private final Field2d field; // for visualization

  // One entry per camera.
  private final Map<String, Cam> cams = new LinkedHashMap<>();

  // --- Simulation-only members ---
  private VisionSystemSim visionSim; // “the world” (targets + cameras)

  // Used to disable vision for very beginning of match
  private double visionPauseUntil = 0.0;

  public Vision(Supplier<Pose2d> currentPoseSupplier, Field2d field) {
    this.currentPoseSupplier = currentPoseSupplier;
    this.field = field;

    // Build a camera + estimator for each configured camera in Constants.Vision.CAMERAS
    for (CameraConfig cfg : VisionConstants.CAMERAS) {
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

    // ---- SIM INIT ----
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(Constants.APRIL_TAG_FIELD_LAYOUT);

      for (CameraConfig cfg : VisionConstants.CAMERAS) {
        Cam cam = cams.get(cfg.name);

        // Camera properties (FOV, res, FPS)
        cam.camProps = new SimCameraProperties();
        cam.camProps.setCalibration(960, 720,      // width, height
                                    Rotation2d.fromDegrees(70.0));  // diagonal FOV approx
        cam.camProps.setFPS(30);
        cam.camProps.setAvgLatencyMs(35);
        cam.camProps.setLatencyStdDevMs(5);

        cam.cameraSim = new PhotonCameraSim(cam.camera, cam.camProps);
        cam.cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cam.cameraSim, cfg.robotToCam);
      }
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
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    
    if (now < visionPauseUntil) return;   // short quiet period after any reset

    for (Cam cam : cams.values()) {
      if (cam.lastEstimate.isEmpty()) continue;

      EstimatedRobotPose est = cam.lastEstimate.get();
      if (est.timestampSeconds < visionPauseUntil) continue; // drop pre-reset frames
      Matrix<N3, N1> stdDevs =
          (est.targetsUsed.size() >= 2) ? VisionConstants.MULTI_TAG_STD_DEVS : VisionConstants.SINGLE_TAG_STD_DEVS;

      swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
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

  // =================== Internal per-camera structure ===================
  private static class Cam {
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;

    // --- Simulation-only per-camera ---
    PhotonCameraSim cameraSim;
    SimCameraProperties camProps;

    PhotonPipelineResult lastResult = new PhotonPipelineResult();
    Optional<EstimatedRobotPose> lastEstimate = Optional.empty();

    Cam(PhotonCamera camera, PhotonPoseEstimator estimator) {
      this.camera = camera;
      this.estimator = estimator;
    }
  }

  // Helper functions
  public void pauseVisionFor(double seconds) {
    visionPauseUntil = Timer.getFPGATimestamp() + seconds;
  }

  // Call to hard-sync the sim world to a given pose immediately
  public void forceUpdateSimToPose(Pose2d pose) {
    if (visionSim != null) {
      visionSim.update(pose);       // re-anchor the world
    }
    // Clear any stale frames so nothing old gets fused
    for (Cam c : cams.values()) {
      c.lastEstimate = Optional.empty();
      c.lastResult = new PhotonPipelineResult();
    }
  }
}
