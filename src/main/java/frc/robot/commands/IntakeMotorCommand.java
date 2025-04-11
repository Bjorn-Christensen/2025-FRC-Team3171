package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMotorCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public IntakeMotorCommand(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
