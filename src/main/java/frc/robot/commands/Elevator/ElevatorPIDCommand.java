package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends Command{
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final double setpoint;

    public ElevatorPIDCommand(ElevatorSubsystem elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setpoint = setpoint;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            elevatorSubsystem.setMotor(0);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
