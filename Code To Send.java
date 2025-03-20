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