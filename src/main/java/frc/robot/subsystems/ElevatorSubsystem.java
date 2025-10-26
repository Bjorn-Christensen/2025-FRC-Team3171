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
        pidController = elevatorLeaderMotor.getClosedLoopController();
    }

    // Move elevator to desired encoder position
    public void setSetpoint(double setpoint) {
        pidController.setReference(setpoint, ControlType.kPosition);
    }

    // Manual elevator control
    public void setMotor(double speed) {
        // Safety feature to limit bounds of elevator movement
        if ((speed < 0 && getEncoder() <= ElevatorConstants.LOWER_BOUND) ||
            (speed > 0 && getEncoder() >= ElevatorConstants.UPPER_BOUND)) {
            elevatorLeaderMotor.set(0);
        } else {
            elevatorLeaderMotor.set(speed);
        }
    }

    // Toggle elevator motor break to allow the elevator to be easily reset at its base position
    public void setMotorBrake(boolean brake) {
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        SparkMaxConfig tempConfig = new SparkMaxConfig();
        tempConfig.idleMode(mode);
        elevatorLeaderMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        elevatorFollowerMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Getters
    public double getEncoder() { return encoder.getPosition(); }
    public double getP() { return elevatorLeaderMotor.configAccessor.closedLoop.getP(); }
    public double getI() { return elevatorLeaderMotor.configAccessor.closedLoop.getI(); }
    public double getD() { return elevatorLeaderMotor.configAccessor.closedLoop.getD(); }

    // Live Tuning
    public void setPID(double p, double i, double d) {
        SparkMaxConfig tempConfig = new SparkMaxConfig();
        tempConfig.closedLoop.pid(p,i,d);
        elevatorLeaderMotor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
}