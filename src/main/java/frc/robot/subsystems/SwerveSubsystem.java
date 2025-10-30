package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;
    private Vision vision;

    // Precision Scoring PID Controller
    private final HolonomicDriveController precisionHDC;
    private static final double POS_TOL_M = 0.05;   // tolerance in meters
    private static final double ANG_TOL_RAD = Math.toRadians(2.0); // tolerace in degrees

    public SwerveSubsystem(File directory) {

        Pose2d startingPose = new Pose2d();

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivetrainConstants.MAX_SPEED, startingPose); // Initialize SwerveDrive Object
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.

        // Enable vision tracking and path planner
        setupPhotonVision();
        swerveDrive.stopOdometryThread(); // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        setupPathPlanner();

        // Initialize precision movement controller
        precisionHDC = new HolonomicDriveController(
            new PIDController(2.5, 0.0, 0.2),   // X PID (tune on carpet)
            new PIDController(2.5, 0.0, 0.2),   // Y PID
            new ProfiledPIDController(3.0, 0.0, 0.2, // Theta PID (profiled)
                new TrapezoidProfile.Constraints(
                    swerveDrive.getMaximumChassisAngularVelocity(),
                    2.0 * swerveDrive.getMaximumChassisAngularVelocity())));
    }

    // Drive the robot given a chassis field oriented velocity.
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    // Create vision object
    public void setupPhotonVision() {
        vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry(); // When vision is enabled we must manually update odometry in SwerveDrive
        vision.updatePoseEstimation(swerveDrive);

        // Authoritative robot pose for AdvantageScope
        Logger.recordOutput("Robot/Pose", swerveDrive.getPose());
    }

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

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public void zeroGyroWithAlliance() {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            swerveDrive.zeroGyro();
            //Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            swerveDrive.zeroGyro();
        }
    }

    // --------------------------------------------
    // Pathplanner Functions / Autonomous Control
    // --------------------------------------------

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    // Setup the PathPlanner, taken directly from YAGSL, don't change other than PPHolonomicController PIDs
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings.
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                (speedsRobotRelative, moduleFeedForwards) -> {
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces());
                },
                // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PPHolonomicDriveController(
                    new PIDConstants(1.5, 0.0, 0.1), // Translation PID constants
                    new PIDConstants(6.0, 0.0, 0.15) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Preload PathPlanner Path finding
        PathfindingCommand.warmupCommand().schedule();
    }

    // Resets odometry for simulation
    public void primeStartingPose(Pose2d start) {
        // Put odometry at the auto start pose before the match starts
        resetOdometry(start); 
      
        // Tell vision to align
        if (SwerveDriveTelemetry.isSimulation) {
          vision.pauseVisionFor(0.35);    // ~ camera latency + buffer
        }
    }
    
    // Find nearest reef april tag and drive to it
    public Command driveToReef(boolean left) {
        Pose2d tagPose;
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            tagPose = vision.nearestTagFromList(AprilTagConstants.REEF_TAGS_BLUE, swerveDrive);
        } else {
            tagPose = vision.nearestTagFromList(AprilTagConstants.REEF_TAGS_RED, swerveDrive);
        }

        if (tagPose == null) {
            return Commands.print("No tag found!");
        }

        // Transform tagPose into target pose .5 meters away and facing towards the target
        double distLeftOrRight = left ? 0.1 : 0.3;
        Pose2d targetPose = tagPose.transformBy(new Transform2d(0.7, distLeftOrRight, Rotation2d.fromDegrees(180)));
        System.out.println("Target Pose: " + targetPose.toString());
        return driveToPose(targetPose, tagPose, true, left);
    }

    // Find nearest human load april tag and drive to it
    public Command driveToHumanLoad() {
        Pose2d tagPose;
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            tagPose = vision.nearestTagFromList(AprilTagConstants.HUMAN_LOAD_TAGS_BLUE, swerveDrive);
        } else {
            tagPose = vision.nearestTagFromList(AprilTagConstants.HUMAN_LOAD_TAGS_RED, swerveDrive);
        }

        if (tagPose == null) {
            return Commands.print("No tag found!");
        }

        // Transform tagPose into target pose .5 meters away and facing away from the target
        Pose2d targetPose = tagPose.transformBy(new Transform2d(0.3, 0.0, Rotation2d.fromDegrees(0)));
        System.out.println("Target Pose: " + targetPose.toString());
        return driveToPose(targetPose, tagPose, false, false);
    }

    // Drive to desired Pose2d using PathPlanner's Autobuilder function
    public Command driveToPose(Pose2d targetPose, Pose2d tagPose, boolean scoringTarget, boolean left) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(), 4.0,
            swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            // Goal end velocity in meters/sec
            edu.wpi.first.units.Units.MetersPerSecond.of(0))
            .andThen(scoringTarget ? precisionLineUp(left) : Commands.none());
    }

    // Line up precise and slow
    public Command precisionLineUp(boolean left) {

        Pose2d tagPose;
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            tagPose = vision.nearestTagFromList(AprilTagConstants.REEF_TAGS_BLUE, swerveDrive);
        } else {
            tagPose = vision.nearestTagFromList(AprilTagConstants.REEF_TAGS_RED, swerveDrive);
        }

        // Configure tolerances on every start (safe if called multiple times)
        precisionHDC.setTolerance(
            new Pose2d(POS_TOL_M, POS_TOL_M, new Rotation2d(ANG_TOL_RAD))
        );

        // Enable the controller
        precisionHDC.setEnabled(true);

        // Translation: (forward/back along tag X, left/right along tag Y)
        var tx = 0.5;         // e.g., 0.50 m out from the tag face
        var ty = left ? 0.1 : 0.5;        // -left, +right relative to the tag's rotation

        Pose2d targetPose = tagPose.transformBy(new Transform2d(new Translation2d(tx, ty), 
                                                                Rotation2d.fromDegrees(180)));

        return runEnd(
            () -> {

                Pose2d cur = swerveDrive.getPose();
                Rotation2d desiredHeading = targetPose.getRotation();

                ChassisSpeeds commanded =
                    precisionHDC.calculate(cur, targetPose, 0.0, desiredHeading);

                double vx = MathUtil.clamp(commanded.vxMetersPerSecond, -1.0, 1.0);
                double vy = MathUtil.clamp(commanded.vyMetersPerSecond, -1.0, 1.0);
                double omega = MathUtil.clamp(commanded.omegaRadiansPerSecond,
                                            -Units.degreesToRadians(30),
                                            Units.degreesToRadians(30));


                // Convert robot → field using current robot heading, then drive field-oriented
                ChassisSpeeds fieldCmd = ChassisSpeeds.fromRobotRelativeSpeeds(
                    vx, vy, omega, cur.getRotation());
                swerveDrive.driveFieldOriented(fieldCmd);                          
            },
            () -> swerveDrive.lockPose() // onEnd stop move
        )
        // Finish when we're inside the tight pos + angle window
        .until(() -> precisionHDC.atReference());
    }
}
