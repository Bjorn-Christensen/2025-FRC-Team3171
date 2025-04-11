package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

        private final SparkMax climberMotor;


    public ClimberSubsystem() {
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_CAN_ID, ClimberConstants.MOTOR_TYPE);
    }

    public void setMotor(double speed) {
        climberMotor.set(speed);   
    }

    // Shuffleboard / Testing
    private final ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");

    @Override
    public void periodic() {

    }

}
