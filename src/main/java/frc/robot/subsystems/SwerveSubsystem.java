package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants.DrivetrainConstants;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;
    private Vision vision;

    public SwerveSubsystem(File directory) {

        Pose2d startingPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivetrainConstants.MAX_SPEED, startingPose);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(true);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.

        // Enable vision tracking and path planner
        // setupPhotonVision();
        // swerveDrive.stopOdometryThread(); // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        // setupPathPlanner();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    // public void setupPhotonVision() {
    //     vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    // }

    @Override
    public void periodic() {
        // swerveDrive.updateOdometry(); // When vision is enabled we must manually update odometry in SwerveDrive
        // vision.updatePoseEstimation(swerveDrive);
    }

    // public void setupPathPlanner() {
    //     // Load the RobotConfig from the GUI settings.
    //     RobotConfig config;
    //     try {
    //         config = RobotConfig.fromGUISettings();

    //         // Configure AutoBuilder last
    //         AutoBuilder.configure(
    //             this::getPose,
    //             this::resetOdometry,
    //             this::getRobotVelocity,
    //             (speedsRobotRelative, moduleFeedForwards) -> {
    //                     swerveDrive.drive(
    //                         speedsRobotRelative,
    //                         swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
    //                         moduleFeedForwards.linearForces());
    //             },
    //             // PPHolonomicController is the built in path following controller for holonomic drive trains
    //             new PPHolonomicDriveController(
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //             ),
    //             config,
    //             () -> {
    //                 // Boolean supplier that controls when the path will be mirrored for the red alliance
    //                 // This will flip the path being followed to the red side of the field.
    //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                     return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //             },
    //             this // Reference to this subsystem to set requirements
    //         );
    //     } catch (Exception e) {
    //         e.printStackTrace();
    //     }

    //     //Preload PathPlanner Path finding
    //     PathfindingCommand.warmupCommand().schedule();
    // }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    // Drive the robot given a chassis field oriented velocity.
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void zeroGyroWithAlliance() {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            swerveDrive.zeroGyro();
            //Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            swerveDrive.zeroGyro();
        }
    }

    // public Command driveToPose(Pose2d pose) {
    //     // Create the constraints to use while pathfinding
    //     PathConstraints constraints = new PathConstraints(
    //         swerveDrive.getMaximumVelocity(), 4.0,
    //         swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    //     // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //     return AutoBuilder.pathfindToPose(
    //         pose,
    //         constraints,
    //         // Goal end velocity in meters/sec
    //         edu.wpi.first.units.Units.MetersPerSecond.of(0));
    // }

}
