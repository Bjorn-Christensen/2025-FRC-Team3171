package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    
    // Hardware
    private final SparkMax elevatorMotor; 
    private final Encoder encoder;

    // PID
    private final PIDController pidController;

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_CAN_ID, ElevatorConstants.MOTOR_TYPE);
        encoder = new Encoder(ElevatorConstants.ELEVATOR_ENCODER_CHANNEL_A,ElevatorConstants.ELEVATOR_ENCODER_CHANNEL_B);
        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    }
    
    public double getEncoder() {
        return encoder.get();
    }

    public void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public void resetPID() {
        pidController.reset();
    }

    public void setMotor(double speed, boolean isManual) {
        if(!isManual) speed = pidController.calculate(getEncoder());

        if ((speed < 0 && encoder.get() <= ElevatorConstants.LOWER_BOUND) ||
            (speed > 0 && encoder.get() >= ElevatorConstants.UPPER_BOUND)) {
            elevatorMotor.set(0);
        } else {
            elevatorMotor.set(speed);
        }
    }

    // Shuffleboard / Testing
    private final ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private final GenericEntry encoderOutput = tab.add("Encoder Value", 0).getEntry();
    private final GenericEntry speedOutput = tab.add("PID Calculated Speed", 0.0).getEntry();
    // private final GenericEntry kPEntry = tab.add("Enter kP", 0.0).getEntry();
    // private final GenericEntry kIEntry = tab.add("Enter kI", 0.0).getEntry();
    // private final GenericEntry kDEntry = tab.add("Enter kD", 0.0).getEntry();
    // private final GenericEntry heightEntry = tab.add("Desired Height", 0.0).getEntry();
    // private final GenericEntry simEncoderEntry = tab.add("Sim Encoder", 0.0).getEntry();

    @Override
    public void periodic() {
        encoderOutput.setDouble(getEncoder());
        speedOutput.setDouble(pidController.calculate(getEncoder()));

        // pidController.setP(kPEntry.getDouble(ElevatorConstants.kP));
        // pidController.setI(kIEntry.getDouble(ElevatorConstants.kI));
        // pidController.setD(kDEntry.getDouble(ElevatorConstants.kD));
        // pidController.setSetpoint(heightEntry.getDouble(0));
    }
    
}
