package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public final class Constants {
    
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    // Elevator Constants
    public static class ElevatorConstants {
        public static final int ELEVATOR_CAN_ID = 11;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double kP = .1, kI = 0, kD = 0; // PID Values
        public static final int ELEVATOR_ENCODER_CHANNEL_A = 1, ELEVATOR_ENCODER_CHANNEL_B = 0;
        public static final int ELEVATOR_LINE_SENSOR_CHANNEL = 2;
        public static final int LOWER_BOUND = 300, UPPER_BOUND = 11700; // Encoder Min/Max
        public static final double MANUAL_ELEVATOR_SPEED = 0.7;
        public static final int POSITION_ONE = 0, POSITION_TWO = 4143, POSITION_THREE = 7305, POSITION_FOUR = 11561;
        public static final int LOAD_STATION_POSITION = 2690;
    }
   
    // Intake Constants
    public static class IntakeConstants {
        // Intake Motor
        public static final int INTAKE_CAN_ID = 9;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double INTAKE_SPEED = 0.7, OUTTAKE_SPEED = -0.7;

        // Pneumatics
        public static final int PCM_CAN_ID = 21;
        public static final int PICKUP_FORWARD_CHANNEL = 0, PICKUP_REVERSE_CHANNEL = 1;
        public static final int MIN_PRESSURE = 95, MAX_PRESSURE = 110;
    }

    // Climber Constants
    public static class ClimberConstants {
        public static final int CLIMBER_CAN_ID = 13;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double CLIMBER_SPEED = 0.7;
    }

    // Drivetrain Constants
    public static final class DrivetrainConstants {
        public static final double MAX_SPEED = Units.feetToMeters(15.0);
        public static final double WHEEL_LOCK_TIME = 10; // Hold time on motor brakes when disabled (seconds)
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

}
