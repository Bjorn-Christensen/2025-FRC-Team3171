package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Elevator.ElevatorJoystickCommand;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Intake.IntakeMotorCommand;
import frc.robot.commands.Intake.IntakeMoveCommand;
import frc.robot.commands.Sequential.PlaceSequence;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;

import swervelib.SwerveInputStream;

public class RobotContainer {

    // Define robot subsystems and commands
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(DrivetrainConstants.swerveJsonDirectory);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Telemetry
    private final Telemetry telemetry = new Telemetry(elevatorSubsystem, intakeSubsystem, climberSubsystem);

    // Joysticks
    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController operatorXbox = new CommandXboxController(1);

    // Auton Chooser
    private final LoggedDashboardChooser<Command> autoChooser;

    // Subsystem requirements required for commands writting direct within subsystem class
    private final Set<Subsystem> swerveReq = Set.of(swerveSubsystem);

    // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() * -1,
                                                                  () -> driverXbox.getLeftX() * -1)
                                                              .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                              .deadband(DriverConstants.DEADZONE)
                                                              .scaleTranslation(0.8)
                                                              .scaleRotation(0.8)
                                                              .allianceRelativeControl(true);

    // Constructor
    public RobotContainer() {
        configureButtonBindings();

        // Register named commands for pathplanner auton
        NamedCommands.registerCommand("PlaceSequenceL1", 
            new PlaceSequence(elevatorSubsystem, intakeSubsystem, ElevatorConstants.POSITION_TWO));
        NamedCommands.registerCommand("PlaceSequenceL2", 
            new PlaceSequence(elevatorSubsystem, intakeSubsystem, ElevatorConstants.POSITION_THREE));
        NamedCommands.registerCommand("PrecisionScoreLeft", 
            Commands.defer(() -> swerveSubsystem.precisionLineUp(true), Set.of(swerveSubsystem)));
        NamedCommands.registerCommand("PrecisionScoreRight", 
            Commands.defer(() -> swerveSubsystem.precisionLineUp(false), Set.of(swerveSubsystem)));

        // Build chooser after NamedCommands so event markers have something to call
        autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser("Test"));
    }

    private void configureButtonBindings() {

        // Elevator Controls
        operatorXbox.rightTrigger(OperatorConstants.RIGHT_TRIGGER_DEADZONE).whileTrue(new ElevatorJoystickCommand(elevatorSubsystem, operatorXbox));
        operatorXbox.leftTrigger(OperatorConstants.LEFT_TRIGGER_DEADZONE).whileTrue(new ElevatorJoystickCommand(elevatorSubsystem, operatorXbox));
        operatorXbox.a().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_ONE));       
        operatorXbox.b().onTrue(new PlaceSequence(elevatorSubsystem, intakeSubsystem, ElevatorConstants.POSITION_TWO));
        operatorXbox.x().onTrue(new PlaceSequence(elevatorSubsystem, intakeSubsystem, ElevatorConstants.POSITION_THREE));
        operatorXbox.leftBumper().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.LOAD_STATION_POSITION));

        // Intake Controls
        operatorXbox.povUp().onTrue(new IntakeMoveCommand(intakeSubsystem, true));
        operatorXbox.povDown().onTrue(new IntakeMoveCommand(intakeSubsystem, false));
        driverXbox.rightTrigger(0.1).whileTrue(new IntakeMotorCommand(intakeSubsystem, IntakeConstants.INTAKE_SPEED));
        driverXbox.leftTrigger(0.1).whileTrue(new IntakeMotorCommand(intakeSubsystem, IntakeConstants.OUTTAKE_SPEED));
        
        // Climb Controls
        driverXbox.rightBumper().whileTrue(new ClimberCommand(climberSubsystem, ClimberConstants.CLIMBER_SPEED));
        driverXbox.leftBumper().whileTrue(new ClimberCommand(climberSubsystem, -ClimberConstants.CLIMBER_SPEED));

        // Auto Drive Controls
        driverXbox.b().onTrue(Commands.defer(() -> swerveSubsystem.driveToReef(false), swerveReq));
        driverXbox.x().onTrue(Commands.defer(() -> swerveSubsystem.driveToReef(true), swerveReq));
        driverXbox.a().onTrue(Commands.defer(() -> swerveSubsystem.driveToHumanLoad(), swerveReq));

    }

    // Auton chooser called on Autonomous Init
    public Command getAutonomousCommand() {
        Command selected = autoChooser.get();
        if(selected != null) {
            String autoName = selected.getName();
            try {
                var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                if (!paths.isEmpty()) {
                    PathPlannerPath firstPath = paths.get(0);
                    Pose2d startPose = firstPath.getPathPoses().get(0);
                    swerveSubsystem.primeStartingPose(startPose);
                }
            } catch (Exception e) {
                System.err.println("primeStartingPose failed: " + e.getMessage());
            }
        }

        return selected;
    }

    // Set drive motor brakes
    public void setDriveMotorBrake(boolean brake) {
        swerveSubsystem.setMotorBrake(brake);
    }

    // Allows elevator to be dropped to base position while disabled
    public void setElevatorMotorBrake(boolean brake) {
        elevatorSubsystem.setMotorBrake(brake);
    }

    // Set drive Controls For Teleop
    public void setSwerveDefaultCommand() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
    }

    // Runs Live PID Tuning
    public void periodic() {
        telemetry.periodic();
    }
}
