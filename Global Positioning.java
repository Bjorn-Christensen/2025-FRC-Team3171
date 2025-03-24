/// FIND ROBOT POSITION ON FIELD

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;

// On init
AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //AprilTagFields.k2025ReefscapeWelded
PhotonCamera camera = new PhotonCamera("Arducam_USB_Camera");
Transform3d robotToCam = new Transform3d(
    new Translation3d(inchesToMeters(5.49), // X: Forward/Backward from center (+/-)
                      inchesToMeters(8.86), // Y: Left/Right from center (+/-)
                      inchesToMeters(32.49), // Z: Up/Down from center (+/-)
                      ),
    new Rotation3d()
);
boolean targetLocked = false;
boolean closeEnough = false;
double horizontalDiff = 0;
double verticalDiff = 0;
double directDriveAngle = 0;
double trackingMagnitude = 0;
int[] reefTagIDs; // Set Fiducial IDs of AprilTags given alliance side
switch (DriverStation.getAlliance().get()) {
    case Red:
        reefTagIDs = new int[]{6,7,8,9,10,11};
        break;
    default:
        reefTagIDs = new int[]{17,18,19,20,21,22};
        break;
}

// Method Variables
PhotonPipelineResult result = camera.getLatestResult();
PhotonTrackedTarget target;
Transform3d cameraToTag;
double closeDist = 10; // Initialize at "large" values
double closeRot = 90; // (Degrees)

// Poses are relative to bottom-left corner of field with CCW rotation
Pose3d tagPose3D;
Pose3d robotPose3D; 

Pose2d tagPose2D;
Pose2d robotPose2D; 
Pose2d goalPose2D;


if(result.hasTargets()) {
    target =  result.getBestTarget();
    cameraToTag = target.getBestCameraToTarget();
    tagPose3D = fieldLayout.getTagPose(target.getFiducialId).get(); // .get() is necessary to retrieve Pose3d from Optional<Pose3d>
    tagPose2D = tagPose3D.toPose2d();
    robotPose3D = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTag, tagPose3D, robotToCam.inverse());
    robotPose2D = robotPose3D.toPose2d();
    
    // If no target has been chosen, check if seen target is on the reef, then create desired pose
    if(!targetLocked) {
        for(int fiducialId : reefTagIDs) {
            if(target.getFiducialId() == fiducialId) {
                goalPose2D = new Pose2d(
                  tagPose2D.getTranslation().plus(new Translation2d(-1.0, 0.0).rotateBy(tagPose2D.getRotation())), // Move X meters behind Tag
                  tagPose2D.getRotation().plus(Rotation2d.fromDegrees(180)) // Face opposite direction (Towards tag)
                );
                targetLocked = true;
            }
        }
    }

    // Check if estimated pose close enough to desired pose
    if(goalPose2D != null) {
        closeDist = robotPose2D.getTranslation().getDistance(goalPose2D.getTranslation());
        closeRot = Math.abs(robotPose2D.getRotation().minus(goalPose2D.getRotation()).getDegrees());
        if(closeDist < 0.1 && closeRot < 5) closeEnough = true;

        horizontalDiff = robotPose2D.getY() - goalPose2D.getY(); // Field pose -Y corresponds to Controller +X
        verticalDiff = goalPose2D.getX() - robotPose2D.getX(); // Field Pose +X corresponds to Controller +Y
        directDriveAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(horizontalDiff, verticalDiff)));
        trackingMagnitude = 0.3; // TODO:Change to acceleration/deceleration function later if rest of code works
        // Math.max(MathUtil.clamp(robotPose.getTranslation().getDistance(goalPose.getTranslation()), 0.0, 1.0), .1); // linear scale
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + (goalPose2D.getRotation().minus(robotPose2D.getRotation()) / 4)));
    }
  
    if(!closeEnough) {
        // May have issue where position is correct but rotation is not, with constant speed we will move past desired pose
        swerveDrive.drive(directDriveAngle, trackingMagnitude, gyroPIDController.getPIDValue(), false);
    } else if(closeDist < 0.1) {
        swerveDrive.drive(0, 0, gyroPIDController.getPIDValue(), false);
    } else if(closeRot < 5) {
        swerveDrive.drive(directDriveAngle, trackingMagnitude, 0, false);
    }

} else {
    // TODO: Reduce number of times this else statement is called by running AprilTagDetection on rear camera simultaneously
    if(robotPose2D != null && goalPose2D != null) {
        // TODO: Continue on path towards goal based on most recently estimated robotPose
        if(!closeEnough) {
            swerveDrive.drive(directDriveAngle, trackingMagnitude, gyroPIDController.getPIDValue(), false);
        }
    } else {
        // TODO: Either rotate robot until tag detected or do nothing (Driver/Lighting/Cam error)
        // swerveDrive.drive(0, 0, 0.25, false);
    }
}
