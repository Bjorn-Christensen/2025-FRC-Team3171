package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

// Dashboard UI
public final class Telemetry {

    // Tabs
    private final NetworkTable telemRoot = NetworkTableInstance.getDefault().getTable("Telemetry");
    private final NetworkTable tuningRoot = NetworkTableInstance.getDefault().getTable("Tuning");

    // Publishers 
    private final DoublePublisher elevEncoderPub;
    private final DoublePublisher intakePressurePub;
    private final StringPublisher intakeStatePub;
    private final DoublePublisher climberSpeedPub;

    // Subscribers / For live tuning
    private final DoubleSubscriber elev_kP_sub;
    private final DoubleSubscriber elev_kI_sub;
    private final DoubleSubscriber elev_kD_sub;;

    // Cache last-applied PID to avoid spamming setPID
    private double last_kP, last_kI, last_kD;

    // Subsystems
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;

    public Telemetry(ElevatorSubsystem elevator, IntakeSubsystem intake, ClimberSubsystem climber) {
        this.elevator = elevator;
        this.intake = intake;
        this.climber = climber;

        // --- Publishers ---
        elevEncoderPub = telemRoot.getSubTable("Elevator").getDoubleTopic("Encoder").publish();
        intakePressurePub = telemRoot.getSubTable("Intake").getDoubleTopic("PneumaticPressurePsi").publish();
        intakeStatePub = telemRoot.getSubTable("Intake").getStringTopic("PneumaticState").publish();
        climberSpeedPub = telemRoot.getSubTable("Climber").getDoubleTopic("MotorSpeed").publish();

        // --- Subscribers for tuning ---
        // Seed topics with current values so users see defaults immediately
        var elevTuningTbl = tuningRoot.getSubTable("Elevator");
        elevTuningTbl.getDoubleTopic("kP").publish().set(elevator.getP());
        elevTuningTbl.getDoubleTopic("kI").publish().set(elevator.getI());
        elevTuningTbl.getDoubleTopic("kD").publish().set(elevator.getD());

        elev_kP_sub = elevTuningTbl.getDoubleTopic("kP").subscribe(elevator.getP());
        elev_kI_sub = elevTuningTbl.getDoubleTopic("kI").subscribe(elevator.getI());
        elev_kD_sub = elevTuningTbl.getDoubleTopic("kD").subscribe(elevator.getD());

        // Initialize last values
        last_kP = elevator.getP();
        last_kI = elevator.getI();
        last_kD = elevator.getD();
    }

    public void periodic() {
        // ---- Publish telemetry ----
        elevEncoderPub.set(elevator.getEncoder());
        intakePressurePub.set(intake.getCompressorPressure());
        intakeStatePub.set(intake.getPneumaticState());
        climberSpeedPub.set(climber.getSpeed());

        // ---- Live PID tuning ----
        double kP = elev_kP_sub.get();
        double kI = elev_kI_sub.get();
        double kD = elev_kD_sub.get();

        if (kP != last_kP || kI != last_kI || kD != last_kD) {
            elevator.setPID(kP, kI, kD);
            last_kP = kP; last_kI = kI; last_kD = kD;
            System.out.printf("Updated Elevator PID: P=%.3f I=%.3f D=%.3f%n", kP, kI, kD);
        }
    }
}