package frc.robot.subsystems;

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
        climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80).secondaryCurrentLimit(100).voltageCompensation(12.0);
        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Manual climber control
    public void setMotor(double speed) {
        climberMotor.setVoltage(speed * 12.0); // Speed: [-1,1], use setVoltage() over set() for max and consistent speed under torque
    }

    // Getters
    public double getSpeed() { return climberMotor.get(); }

}