package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJoystickCommand extends Command{
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final CommandXboxController xboxController;

    public ElevatorJoystickCommand(ElevatorSubsystem elevatorSubsystem, CommandXboxController xboxController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.xboxController= xboxController;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = xboxController.getRightTriggerAxis() - xboxController.getLeftTriggerAxis();
        elevatorSubsystem.setMotor(speed);
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
