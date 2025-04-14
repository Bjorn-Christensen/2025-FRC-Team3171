package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    
    // Hardware
    private final SparkMax elevatorMotor;
    private final RelativeEncoder encoder; 

    // Onboard PID
    private final SparkClosedLoopController pidController;

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_CAN_ID, ElevatorConstants.MOTOR_TYPE);
        encoder = elevatorMotor.getEncoder();
        pidController = elevatorMotor.getClosedLoopController();
    }
    
    public double getEncoder() {
        return encoder.getPosition();
    }

    public void setSetpoint(double setpoint) {
        if ((setpoint < ElevatorConstants.LOWER_BOUND && getEncoder() <= ElevatorConstants.LOWER_BOUND) ||
            (setpoint > ElevatorConstants.UPPER_BOUND && getEncoder() >= ElevatorConstants.UPPER_BOUND)) {
            elevatorMotor.set(0);
        } else {
            pidController.setReference(setpoint, ControlType.kPosition);
        }
    }

    public void resetPID() {
        encoder.setPosition(0);
    }

    public void setMotor(double speed) {
        elevatorMotor.set(speed);
    }

    // Shuffleboard / Testing
    private final ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private final GenericEntry encoderOutput = tab.add("Elevator Encoder Value", 0).getEntry();
    private final GenericEntry speedOutput = tab.add("Elevator PID Calculated Speed", 0.0).getEntry();
    private final GenericEntry curMotorSpeed = tab.add("Elevator Motor Speed", 0.0).getEntry();

    @Override
    public void periodic() {
        encoderOutput.setDouble(getEncoder());
        speedOutput.setDouble(elevatorMotor.getAppliedOutput());
        curMotorSpeed.setDouble(elevatorMotor.get());
    }
    
}
