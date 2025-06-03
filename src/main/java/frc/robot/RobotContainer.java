package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Elevator.ElevatorHoldCommand;
import frc.robot.commands.Elevator.ElevatorJoystickCommand;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Intake.IntakeMotorCommand;
import frc.robot.commands.Intake.IntakeMoveCommand;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;

import swervelib.SwerveInputStream;

public class RobotContainer {

    // Define robot subsystems and commands
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Joysticks
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() * -1,
                                                                  () -> driverXbox.getLeftX() * -1)
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                              .deadband(DriverConstants.DEADZONE)
                                                              .scaleTranslation(0.8)
                                                              .scaleRotation(0.8)
                                                              .allianceRelativeControl(true);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        // Elevator Controls
        operatorXbox.rightTrigger(OperatorConstants.RIGHT_TRIGGER_DEADZONE).whileTrue(new ElevatorJoystickCommand(elevatorSubsystem, operatorXbox));
        operatorXbox.leftTrigger(OperatorConstants.LEFT_TRIGGER_DEADZONE).whileTrue(new ElevatorJoystickCommand(elevatorSubsystem, operatorXbox));
        operatorXbox.a().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_ONE));
        operatorXbox.b().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_TWO));
        operatorXbox.y().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_THREE));
        operatorXbox.x().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_FOUR));
        operatorXbox.leftBumper().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.LOAD_STATION_POSITION));
        elevatorSubsystem.setDefaultCommand(new ElevatorHoldCommand(elevatorSubsystem));

        // Intake Controls
        operatorXbox.povUp().onTrue(new IntakeMoveCommand(intakeSubsystem, true));
        operatorXbox.povDown().onTrue(new IntakeMoveCommand(intakeSubsystem, false));
        driverXbox.rightTrigger(0.1).whileTrue(new IntakeMotorCommand(intakeSubsystem, IntakeConstants.INTAKE_SPEED));
        driverXbox.leftTrigger(0.1).whileTrue(new IntakeMotorCommand(intakeSubsystem, IntakeConstants.OUTTAKE_SPEED));
        
        // Climb Controls
        driverXbox.rightBumper().whileTrue(new ClimberCommand(climberSubsystem, ClimberConstants.CLIMBER_SPEED));
        driverXbox.leftBumper().whileTrue(new ClimberCommand(climberSubsystem, -ClimberConstants.CLIMBER_SPEED));

        // Drive Controls
        Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    }

    // TODO: Make Auton Chooser
    public Command getAutonomousCommand() {
        return swerveSubsystem.getAutonomousCommand("TODO");
    }

    public void setMotorBrake(boolean brake) {
        swerveSubsystem.setMotorBrake(brake);
    }
}
