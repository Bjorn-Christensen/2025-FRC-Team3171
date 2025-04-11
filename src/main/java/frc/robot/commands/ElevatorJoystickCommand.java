package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJoystickCommand extends Command{
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorJoystickCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevatorSubsystem.setMotor(speed, true);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
