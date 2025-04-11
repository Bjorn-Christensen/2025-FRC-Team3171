package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMoveCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private final boolean forward;

    public IntakeMoveCommand(IntakeSubsystem intakeSubsystem, boolean forward) {
        this.intakeSubsystem = intakeSubsystem;
        this.forward = forward;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.moveIntake(forward);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.moveIntake(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
