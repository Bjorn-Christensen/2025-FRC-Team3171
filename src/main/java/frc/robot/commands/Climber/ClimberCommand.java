package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{
    
    private final ClimberSubsystem climberSubsystem;
    private final double speed;

    public ClimberCommand(ClimberSubsystem climberSubsystem, double speed) {
        this.climberSubsystem = climberSubsystem;
        this.speed = speed;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
