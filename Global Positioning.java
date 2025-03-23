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
                      inchesToMeters(32.49), // Z: Up/Down from center (+/-), robotToCamRot);
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
                  tagPose2D.getTranslation().plus(new Translation3d(-1.0, 0.0).rotateBy(tagPose2D.getRotation())), // Move X meters behind Tag
                  tagPose2D.getRotation().plus(Rotation2d.fromDegrees(180)) // Face opposite direction (Towards tag)
                );
                targetLocked = true;
            }
        }
    }

    // Check if estimated pose close enough to desired pose
    double closeDist = robotPose2D.getTranslation().getDistance(goalPose2D.getTranslation());
    double closeRot = Math.abs(robotPose2D.getRotation().minus(goalPose2D.getRotation()).getDegrees());
    if(closeDist < 0.1 && closeRot < 5) closeEnough = true;
  
    horizontalDiff = robotPose2D.getY() - goalPose2D.getY(); // Field pose -Y corresponds to Controller +X
    verticalDiff = goalPose2D.getX() - robotPose2D.getX(); // Field Pose +X corresponds to Controller +Y
    directDriveAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(horizontalDiff, verticalDiff)));
    trackingMagnitude = 0.3; // TODO:Change to acceleration/deceleration function later if rest of code works
    // Math.min(MathUtil.clamp(robotPose.getTranslation().getDistance(goalPose.getTranslation()), 0.0, 1.0), .1); // linear scale
    gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + (goalPose2D.getRotation().minus(robotPose2D.getRotation()) / 2)));

    if(!closeEnough) {
        // May have issue where position is correct but rotation is not, with constant speed we will move past desired pose
        swerveDrive.drive(directDriveAngle, trackingMagnitude, gyroPIDController.getPIDValue(), false);
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
    }
}

























































/***
 * Method to retrieve closest April Tag from reef
***/


// Method to retrieve closest April Tag (Sort tags by largest area)
public PhotonAprilTagTarget getNearestReefAprilTag(final int[] reefTags, final String photonCameraName) {
  final HashMap<Integer, PhotonAprilTagTarget> targetData = new HashMap<Integer, PhotonAprilTagTarget>();
  // Query photonCamera for all visible April Tags
  PhotonCamera photonCamera = PHOTON_CAMERAS.get(photonCameraName);
  if (photonCamera != null) {
      if (photonCamera.isConnected()) {
          PhotonPipelineResult result = photonCamera.getLatestResult();
          if (result.hasTargets()) {
              result.targets.forEach((target) -> {
                  PhotonAprilTagTarget existingTarget = targetData.get(target.getFiducialId());
                  if (existingTarget != null) {
                      if (target.getArea() > existingTarget.getPHOTON_TRACKED_TARGET().getArea()) {
                          targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                      }
                  } else {
                      targetData.put(target.getFiducialId(), new PhotonAprilTagTarget(photonCameraName, target));
                  }
              });
          }
      }
  }

  PhotonAprilTagTarget preferredTarget = null;
  for (int fiducialId : reefTags) {
      PhotonAprilTagTarget currentTarget = targetData.get(fiducialId);
      if (preferredTarget == null && currentTarget != null) {
          preferredTarget = currentTarget;
      } else if (currentTarget != null) {
          preferredTarget = preferredTarget.getPHOTON_TRACKED_TARGET().getArea() < currentTarget.getPHOTON_TRACKED_TARGET().getArea() ? currentTarget
                  : preferredTarget;
      }
  }

  return preferredTarget;
}














/***
 * Updated drive with auto lock and scoring
***/

private void driveControlsPeriodic(final XboxControllerState driveControllerState, final double gyroValue) {
  // Get the needed joystick values after calculating the deadzones
  final double leftStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
  final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
  final double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

  // Calculate the left stick angle and magnitude
  final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
  double leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
  leftStickMagnitude = leftStickMagnitude > 1 ? 1 : leftStickMagnitude;

  // Calculate the field corrected drive angle
  final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

  // Desired Targets
  final PhotonAprilTagTarget desiredTagLeft = null;
  final PhotonAprilTagTarget desiredTagRight = null;
  // Drive Controls
  final boolean targetLockLeft = driveControllerState.getXButton();
  if (!targetLockLeft && desiredTagLeft != null) desiredTagLeft = null;
  final boolean targetLockRight = driveControllerState.getBButton();
  if (!targetLockRight && desiredTagRight != null) desiredTagRight = null;
  
  if (rightStickX != 0 || robotOffGround) {
    // Manual turning
    gyroPIDController.disablePID();
    swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, false);
  } else if (targetLockLeft) {
    // Enable April Tag Target Locking
    gyroPIDController.enablePID();

    // Set April Tag Target 
    final PhotonAprilTagTarget aprilTagTarget;
    if(desiredTagLeft == null) {
      // Find nearest reef April Tag
      switch (DriverStation.getAlliance().get()) {
        case Red:
          aprilTagTarget = visionController.getNearestReefAprilTag(new int[] { 6,7,8,9,10,11 }, "Arducam_USB_Camera");
          break;
        default:
          aprilTagTarget = visionController.getNearestReefAprilTag(new int[] { 17,18,19,20,21,22 }, "Arducam_USB_Camera");
          break;
      }
      if (aprilTagTarget != null) {
        // Set desired April Tag and Adjust the gyro lock to point torwards the target
        desiredTagLeft = aprilTagTarget;
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + (aprilTagTarget.getPHOTON_TRACKED_TARGET().getYaw() / 2)));
      }
    } else {
      // Make sure desiredTag is still within vision
      switch (DriverStation.getAlliance().get()) {
        case Red:
          aprilTagTarget = visionController.getAllVisibleAprilTagsByPriority(new int[] { desiredTagLeft.getPHOTON_TRACKED_TARGET().getFiducialId() }, "Arducam_USB_Camera");
          break;
        default:
          aprilTagTarget = visionController.getAllVisibleAprilTagsByPriority(new int[] { desiredTagLeft.getPHOTON_TRACKED_TARGET().getFiducialId() }, "Arducam_USB_Camera");
          break;
      }
      // Adjust the gyro lock to point torwards the target if desired target is still seen
      if (aprilTagTarget != null) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + (aprilTagTarget.getPHOTON_TRACKED_TARGET().getYaw() / 2)));
      }
    }

    // TODO Set magnitude (speed) to scale with distance to target
    // Calculate a translation from the camera to the target.
    // translation.getX() X is normal to april tag plane (Coming out of april tag means we get negative X)
    // translation.getY() means Left-Right

    double CAMERAHEIGHT, APRILTAGHEIGHT = TBD, 0.22225;
    double CAMERAPITCH, APRILTAGPITCH = TBD, aprilTagTarget.getPHOTON_TRACKED_TARGET.getPitch();
    double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(CAMERAHEIGHT, APRILTAGHEIGHT, 
      CAMERAPITCH, APRILTAGPITCH);
    Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(distanceMeters, 
      Rotation2d.fromDegrees(-aprilTagTarget.getPHOTON_TRACKED_TARGET.getYaw()));

    double trackingMagnitude = 0
    if(translation.getX() > .5) {
      trackingMagnitude = Math.min(0.6 * (1 - Math.exp(-0.1 * (translation.getX() - 1))), 0.6);
    }

    leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(translation.getX(), translation.getY())));
    double adjustedDriveAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;
    
    swerveDrive.drive(adjustedDriveAngle, trackingMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, false);

    swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, false);

  } else if (targetLockRight) {
    // Enable April Tag Target Locking
    gyroPIDController.enablePID();

    // Set April Tag Target 
    final PhotonAprilTagTarget aprilTagTarget;
    if(desiredTagRight == null) {
      // Find nearest reef April Tag
      switch (DriverStation.getAlliance().get()) {
        case Red:
          aprilTagTarget = visionController.getNearestReefAprilTag(new int[] { 6,7,8,9,10,11 }, "Arducam_USB_Camera");
          break;
        default:
          aprilTagTarget = visionController.getNearestReefAprilTag(new int[] { 17,18,19,20,21,22 }, "Arducam_USB_Camera");
          break;
      }
      if (aprilTagTarget != null) {
        // Set desired April Tag and Adjust the gyro lock to point torwards the target
        desiredTagRight = aprilTagTarget;
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + (aprilTagTarget.getPHOTON_TRACKED_TARGET().getYaw() / 2)));
      }
    } else {
      // Make sure desiredTag is still within vision
      switch (DriverStation.getAlliance().get()) {
        case Red:
          aprilTagTarget = visionController.getAllVisibleAprilTagsByPriority(new int[] { desiredTagRight.getPHOTON_TRACKED_TARGET().getFiducialId() }, "Arducam_USB_Camera");
          break;
        default:
          aprilTagTarget = visionController.getAllVisibleAprilTagsByPriority(new int[] { desiredTagRight.getPHOTON_TRACKED_TARGET().getFiducialId() }, "Arducam_USB_Camera");
          break;
      }
      // Adjust the gyro lock to point torwards the target if desired target is still seen
      if (aprilTagTarget != null) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue + (aprilTagTarget.getPHOTON_TRACKED_TARGET().getYaw() / 2)));
      }
    }
    
    swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, false);


  }  else {
    // Normal gyro locking
    gyroPIDController.enablePID();

    // Quick Turning
    if (driveControllerState.getPOV() != -1) {
      gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPOV()));
    }

    final boolean closeEnough = Math.abs(Get_Gyro_Displacement(gyroValue, gyroPIDController.getSensorLockValue())) <= 1;
    swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? (closeEnough ? 0 : gyroPIDController.getPIDValue()) : 0, boostMode);
  }
}












/// ALT CODE

private void driveControlsPeriodic(final XboxControllerState driveControllerState, final double gyroValue) {
  // Get the needed joystick values after calculating the deadzones
  final double leftStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
  final double leftStickY = Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
  final double rightStickX = Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
  
  // Calculate the left stick angle and magnitude
  final double leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
  double leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
  leftStickMagnitude = leftStickMagnitude > 1 ? 1 : leftStickMagnitude;

  // Calculate the field corrected drive angle
  final double fieldCorrectedAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;

  // Drive Controls
  final boolean targetLockLeft = driveControllerState.getXButton();
  final boolean targetLockRight = driveControllerState.getBButton();
  
  if (rightStickX != 0 || robotOffGround) {
    // Manual turning
    gyroPIDController.disablePID();
    swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, false);
  } else if (targetLockLeft) { 
    // Enable April Tag Target Locking
    gyroPIDController.enablePID();

    // Create a PhotonCamera object & retrieve latest result
    PhotonCamera camera = new PhotonCamera("Arducam_USB_Camera");
    PhotonPipelineResult result = camera.getLatestResult();

    PhotonTrackedTarget target = null;
    if (result.hasTargets()) {
      // Get the best target and extract pose
      target =  result.getBestTarget();
      Transform3d cameraToTag = target.getBestCameraToTarget();
      Translation3d translation = cameraToTag.getTranslation();

      gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue 
        + (target.getYaw() / 2)));
      // This happens when Yaw is about equal to 0
      final boolean closeEnough = Math.abs(Get_Gyro_Displacement(gyroValue, gyroPIDController.getSensorLockValue())) <= 1;
      // We want to seen Yaw to equal Yaw + or - some value based on distance we want to be left or right of the april tag
      // We lie and say that the april tag is actually left or right x centimeters from where it actually is

     
       if(targetTrackLeft) {
          // distanceLeft is in meters left of center of april tag
          double distanceLeft = .2;
          double desiredYaw = Math.atan2(translation.getY() - distanceLeft, translation.getX());
          gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(gyroValue 
              + (desiredYaw / 2)));
       }

      double trackingMagnitude = 0
      if(translation.getX() > .5) {
        trackingMagnitude = Math.min(0.8 * (1 - Math.exp(-0.1 * (translation.getX() - 1))), 0.8);
      }

      // distanceLeft is a number in meters we want to be left of april tag
      double distanceLeft = .2;
      leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(translation.getX()/10, (translation.getY() - distanceLeft)/10)));
      double adjustedDriveAngle = FIELD_ORIENTED_SWERVE ? Normalize_Gryo_Value(leftStickAngle - gyroValue) : leftStickAngle;
      
      swerveDrive.drive(adjustedDriveAngle, trackingMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, false);

    } else {
      // No tarets seen, drive normally
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, FIELD_ORIENTED_SWERVE ? gyroPIDController.getPIDValue() : 0, false);
    }
  } 
}




// private Pose3d getFieldToRobot(Pose3d fieldToTag, Transform3d cameraToTag, Transform3d robotToCam) {
//     return fieldToTag.transformBy(cameraToTag.inverse()).transformBy(robotToCamera.inverse());
// }

// SwerveDrivePoseEstimator3d poseEst = new SwerveDrivePoseEstimator3d(kinematics, initialGryoAngle, initialModuleStates, initialPose);
// poseEst.addVisionMeasurement(observation.estFieldPose, observation.time);
// poseEst.update(curRawGyroAngle, curModulePositions);
// curEstPose = poseEst.getEstimatedPosition();










