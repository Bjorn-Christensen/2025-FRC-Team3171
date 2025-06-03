package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    
    // Hardware
    private final SparkMax elevatorLeaderMotor;
    private final SparkMax elevatorFollowerMotor;
    private final SparkMaxConfig elevatorLeaderConfig;
    private final SparkMaxConfig elevatorFollowerConfig;
    private final RelativeEncoder encoder; 

    // Onboard PID
    private final SparkClosedLoopController pidController;

    public ElevatorSubsystem() {
        // Initialize Motors
        elevatorLeaderMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEADER_CAN_ID, ElevatorConstants.MOTOR_TYPE);
        elevatorFollowerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_CAN_ID, ElevatorConstants.MOTOR_TYPE);
        // Initialize Motor Configurations
        elevatorLeaderConfig = new SparkMaxConfig();
        elevatorLeaderConfig.idleMode(IdleMode.kBrake).
                            smartCurrentLimit(40).
                            closedLoop.p(ElevatorConstants.kP).i(ElevatorConstants.kI).d(ElevatorConstants.kD);
        elevatorFollowerConfig = new SparkMaxConfig();
        elevatorFollowerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).follow(elevatorLeaderMotor, true);
        // Configure Motors
        elevatorLeaderMotor.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorFollowerMotor.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Initialize Encoder and PID Controller
        encoder = elevatorLeaderMotor.getEncoder();
        encoder.setPosition(0.0); // Encoder Zeroing on Startup
        pidController = elevatorLeaderMotor.getClosedLoopController();
    }
    
    public double getEncoder() {
        return encoder.getPosition();
    }

    public void setSetpoint(double setpoint) {
        // Move elevator to desired encoder position
        pidController.setReference(setpoint, ControlType.kPosition);
    }

    public void setMotor(double speed) {
        // Safety feature to limit bounds of elevator movement
        if ((speed < 0 && getEncoder() <= ElevatorConstants.LOWER_BOUND) ||
            (speed > 0 && getEncoder() >= ElevatorConstants.UPPER_BOUND)) {
            elevatorLeaderMotor.set(0);
        } else {
            elevatorLeaderMotor.set(speed);
        }
    }

    // Shuffleboard / Testing
    private final ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private final GenericEntry encoderOutput = tab.add("Elevator Encoder Value", 0.0).getEntry();
    private final GenericEntry speedOutput = tab.add("Elevator PID Calculated Speed", 0.0).getEntry();

    @Override
    public void periodic() {
        encoderOutput.setDouble(getEncoder());
        speedOutput.setDouble(elevatorLeaderMotor.get());
    }
    
}
