package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.util.List;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public final class Constants {
    
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    // Elevator Constants
    public static class ElevatorConstants {
        public static final int ELEVATOR_LEADER_CAN_ID = 12, ELEVATOR_FOLLOWER_CAN_ID = 11;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double kP = .1, kI = 0, kD = 0; // PID Values
        public static final int LOWER_BOUND = 10, UPPER_BOUND = 170; // Encoder Min/Max
        public static final double MANUAL_ELEVATOR_SPEED = 0.7;
        public static final int POSITION_ONE = 15, POSITION_TWO = 69, POSITION_THREE = 120;
        public static final int LOAD_STATION_POSITION = 45;
    }
   
    // Intake Constants
    public static class IntakeConstants {
        // Intake Motor
        public static final int INTAKE_LEADER_CAN_ID = 9, INTAKE_FOLLOWER_CAN_ID = 10;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double INTAKE_SPEED = 0.7, OUTTAKE_SPEED = -0.7;

        // Pneumatics
        public static final int PCM_CAN_ID = 21;
        public static final int PICKUP_FORWARD_CHANNEL = 0, PICKUP_REVERSE_CHANNEL = 1;
        public static final int MIN_PRESSURE = 95, MAX_PRESSURE = 115;
    }

    // Climber Constants
    public static class ClimberConstants {
        public static final int CLIMBER_CAN_ID = 13;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double CLIMBER_SPEED = 0.7;
    }

    // Drivetrain Constants
    public static final class DrivetrainConstants {
        public static final double MAX_SPEED = Units.feetToMeters(15.5); // Theoretically: ~19.2-19.8, Real: 15-17
        public static final double WHEEL_LOCK_TIME = 10; // Hold time on motor brakes when disabled (seconds)
        public static final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    }

    // Drive Joystick Constants
    public static class DriverConstants {
        // Deadzones (Deadband)
        public static final double DEADZONE = 0.2;
    }

    // Operator Joystick Constants
    public static class OperatorConstants {
        // Deadzones (Deadband)
        public static final double LEFT_TRIGGER_DEADZONE = 0.1, RIGHT_TRIGGER_DEADZONE = 0.1;
    }

    // Vision setup
    public static class Vision {
        // One entry per camera you have configured in PhotonVision
        public static final List<CameraConfig> CAMERAS = List.of(
            new CameraConfig(
                "frontCam",
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0))
            ),
            new CameraConfig(
                "rearCam",
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0))
            )
        );
    }

    // Compact config holder for each Photon camera
    public static final class CameraConfig {
        public final String name;
        public final Transform3d robotToCam;

        public CameraConfig(String name, Transform3d robotToCam) {
            this.name = name;
            this.robotToCam = robotToCam;
        }
    }

}
