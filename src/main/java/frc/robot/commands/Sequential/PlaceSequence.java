package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Intake.IntakeMotorCommand;
import frc.robot.commands.Intake.IntakeMoveCommand;
import frc.robot.subsystems.*;

public class PlaceSequence extends SequentialCommandGroup{

    public PlaceSequence(ElevatorSubsystem elevator, IntakeSubsystem intake, double setpoint) {
        addCommands(
            new ParallelCommandGroup(
                new ElevatorPIDCommand(elevator, setpoint),
                new IntakeMoveCommand(intake, true)
            ),
            new WaitCommand(1.5),
            new IntakeMotorCommand(intake, -.6).withTimeout(1.0),
            new IntakeMoveCommand(intake, false),
            new WaitCommand(.75),
            new ElevatorPIDCommand(elevator, ElevatorConstants.LOAD_STATION_POSITION)
        );
    }
}
