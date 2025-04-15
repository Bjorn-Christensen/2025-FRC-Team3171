package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    // Hardware
    private final SparkMax climberMotor;
    private final SparkMaxConfig climberConfig;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_CAN_ID, ClimberConstants.MOTOR_TYPE);
        climberConfig = new SparkMaxConfig();
        climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).secondaryCurrentLimit(80);
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotor(double speed) {
        climberMotor.set(speed);   
    }

    // Shuffleboard / Testing
    private final ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private final GenericEntry speedOutput = tab.add("Climber Speed", 0.0).getEntry();

    @Override
    public void periodic() {
        speedOutput.setDouble(climberMotor.get());
    }

}
