package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    
    // Hardware
    private final SparkMax intakeMotor;
    private final DoubleSolenoid doubleSolenoid;
    private final Compressor compressor; 

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_CAN_ID, IntakeConstants.MOTOR_TYPE);
        doubleSolenoid = new DoubleSolenoid(IntakeConstants.PCM_CAN_ID, PneumaticsModuleType.REVPH, 
                                            IntakeConstants.PICKUP_FORWARD_CHANNEL, IntakeConstants.PICKUP_REVERSE_CHANNEL);
        compressor = new Compressor(IntakeConstants.PCM_CAN_ID, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(IntakeConstants.MIN_PRESSURE, IntakeConstants.MAX_PRESSURE);
    }

    public void setMotor(double speed) {
        intakeMotor.set(speed);
    }

    public void moveIntake(boolean forward) {
        if(forward) {
            doubleSolenoid.set(Value.kForward);
        } else {
            doubleSolenoid.set(Value.kReverse);
        }
    }

    // Shuffleboard / Testing
    private final ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    private final GenericEntry pneumaticsOutput = tab.add("Air Pressure", 0.0).getEntry();
    private final GenericEntry solenoidOutput = tab.add("Solenoid Value", "").getEntry();

    @Override
    public void periodic() {
        pneumaticsOutput.setDouble(compressor.getPressure());
        solenoidOutput.setString(doubleSolenoid.get().toString());
    }
    
}
