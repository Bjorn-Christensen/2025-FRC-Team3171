package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Robot;
import swervelib.SwerveDrive;
import frc.robot.Constants;

public class Vision {

    public VisionSystemSim visionSim;
    private Supplier<Pose2d> currentPose;
    private Field2d field2d;

    public Vision(Supplier<Pose2d> currentPose, Field2d field) {
        this.currentPose = currentPose;
        this.field2d = field;

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(Constants.APRIL_TAG_FIELD_LAYOUT);

            for (Cameras c : Cameras.values()) {
                c.addToVisionSim(visionSim);
            }
        }
    }

    // Calculates pose relative to an AprilTag on the field
    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + Constants.APRIL_TAG_FIELD_LAYOUT.toString());
        }
    }

    // Update the pose estimation inside of {@link SwerveDrive} with all of the given poses
    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        for (Cameras camera : Cameras.values()) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
            if (poseEst.isPresent()) {
                var pose = poseEst.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                                 pose.timestampSeconds,
                                                 camera.curStdDevs);
            }
        }
    }

    // Generates the estimated robot pose. Returns empty if no Pose Estimates could be generated or were innacurate
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
        Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
        if (Robot.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            // Uncomment to enable outputting of vision targets in sim.
            poseEst.ifPresentOrElse(
                est ->
                    debugField
                        .getObject("VisionEstimation")
                        .setPose(est.estimatedPose.toPose2d()),
                () -> {
                    debugField.getObject("VisionEstimation").setPoses();
                });
        }
        return poseEst;
    }

    // Get distance of the robot from the AprilTag pose
    public double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(id);
        return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

    // Get tracked target from a camera of AprilTagID
    public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
        PhotonTrackedTarget target = null;
        for (PhotonPipelineResult result : camera.resultsList) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget i : result.getTargets()) {
                    if (i.getFiducialId() == id) {
                        return i;
                    }
                }
            }
        }
        return target;
    }

    // Get vision simulation
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    // Update the {@link Field2d} to include tracked targets
    public void updateVisionField() {

        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for (Cameras c : Cameras.values()) {
            if (!c.resultsList.isEmpty()) {
                PhotonPipelineResult latest = c.resultsList.get(0);
                if (latest.hasTargets()) {
                    targets.addAll(latest.targets);
                }
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            if (Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.getFiducialId()).isPresent()) {
                Pose2d targetPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d();
                poses.add(targetPose);
            }
        }

        field2d.getObject("tracked targets").setPoses(poses);
    }

    // Camera Enum to select each camera
    enum Cameras {

        LEFT_CAM("left",
                 new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
                 new Translation3d(Units.inchesToMeters(12.056),
                                   Units.inchesToMeters(10.981),
                                   Units.inchesToMeters(8.44)),
                 VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),

        RIGHT_CAM("right",
                  new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
                  new Translation3d(Units.inchesToMeters(12.056),
                                    Units.inchesToMeters(-10.981),
                                    Units.inchesToMeters(8.44)),
                  VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),

        CENTER_CAM("center",
                   new Rotation3d(0, Units.degreesToRadians(18), 0),
                   new Translation3d(Units.inchesToMeters(-4.628),
                                     Units.inchesToMeters(-10.687),
                                     Units.inchesToMeters(16.129)),
                   VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

        public final  Alert latencyAlert;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        private final Matrix<N3, N1> singleTagStdDevs;
        private final Matrix<N3, N1> multiTagStdDevs;
        private final Transform3d robotToCamTransform;
        public Matrix<N3, N1> curStdDevs;
        public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        public PhotonCameraSim cameraSim;
        public List<PhotonPipelineResult> resultsList = new ArrayList<>();
        private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

        Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
                Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
            latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

            camera = new PhotonCamera(name);

            robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

            poseEstimator = new PhotonPoseEstimator(Constants.APRIL_TAG_FIELD_LAYOUT,
                                                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                    robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevsMatrix;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                // A 640 x 480 camera with a 100 degree diagonal FOV.
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
                // Approximate detection noise with average and standard deviation error in pixels.
                cameraProp.setCalibError(0.25, 0.08);
                // Set the camera image capture framerate (Note: this is limited by robot loop rate).
                cameraProp.setFPS(30);
                // The average and standard deviation in milliseconds of image data latency.
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);

                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }

        // Add camera to {@link VisionSystemSim} for simulated photon vision.
        public void addToVisionSim(VisionSystemSim systemSim) {
            if (Robot.isSimulation()) {
                systemSim.addCamera(cameraSim, robotToCamTransform);
            }
        }

        // Get the result with the least ambiguity from the best tracked target within the Cache
        public Optional<PhotonPipelineResult> getBestResult() {
            if (resultsList.isEmpty()) {
                return Optional.empty();
            }

            PhotonPipelineResult bestResult = resultsList.get(0);
            double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
            double currentAmbiguity = 0;
            for (PhotonPipelineResult result : resultsList) {
                currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
                if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
                    bestResult = result;
                    amiguity = currentAmbiguity;
                }
            }
            return Optional.of(bestResult);
        }

        // Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the cache of results
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }

        // Update the latest results cached with a maximum refresh rate of 1req/15ms, and sorts the list by timestamp
        private void updateUnreadResults() {
            double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
            double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
            double debounceTime = Milliseconds.of(15).in(Seconds);
            for (PhotonPipelineResult result : resultsList) {
                mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
            }
            if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
                (currentTimestamp - lastReadTimestamp) >= debounceTime) {
        
                resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
                lastReadTimestamp = currentTimestamp;
                resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                    return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
                    });
                if (!resultsList.isEmpty()) {
                    updateEstimatedGlobalPose();
                }
            }
        }

        // The latest estimated robot pose on the field from vision data
        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var change : camera.getAllUnreadResults()) {
                visionEst = poseEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
            estimatedRobotPose = visionEst;
        }

        // Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
        // on number of tags, estimation strategy, and distance from the tags.
        private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if (estimatedPose.isEmpty()) {
                curStdDevs = singleTagStdDevs;
            } else {
                var estStdDevs = singleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                // Precalculation - see how many tags we found, and calculate an average-distance metric
                for (var tgt : targets) {
                    var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if (tagPose.isEmpty()) {
                        continue;
                    }
                    numTags++;
                    avgDist +=
                        tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }

                if (numTags == 0) {
                    curStdDevs = singleTagStdDevs;
                } else {
                    avgDist /= numTags;
                    if (numTags > 1) {
                        estStdDevs = multiTagStdDevs;
                    } else if (numTags == 1 && avgDist > 4) {
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    } else {
                        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                    }
                    curStdDevs = estStdDevs;
                }
            }
        }
    }
}
