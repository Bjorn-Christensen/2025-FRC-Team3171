package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHoldCommand extends Command{
    
    private final ElevatorSubsystem elevatorSubsystem;
    private double lastEncoderPos;

    public ElevatorHoldCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        lastEncoderPos = elevatorSubsystem.getEncoder();
    }

    @Override
    public void execute() {
        elevatorSubsystem.setSetpoint(lastEncoderPos);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
