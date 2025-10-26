package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

// Dashboard UI
public final class Telemetry {

    // Tabs
    private final ShuffleboardTab subsystemsTab = Shuffleboard.getTab("Subsystems");
    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    // Grouped grid layouts
    private final ShuffleboardLayout elevatorLayout = subsystemsTab.getLayout("Elevator", BuiltInLayouts.kGrid)
                                                                   .withSize(4, 2)
                                                                   .withPosition(0, 0);
    private final ShuffleboardLayout intakeLayout = subsystemsTab.getLayout("Intake", BuiltInLayouts.kGrid)
                                                                 .withSize(4, 2)
                                                                 .withPosition(4, 0);
    private final ShuffleboardLayout climberLayout = subsystemsTab.getLayout("Climber", BuiltInLayouts.kGrid)
                                                                  .withSize(4, 2)
                                                                  .withPosition(0, 2);

    // Optional tuning entries
    private GenericEntry elev_kP, elev_kI, elev_kD;

    // Store previous tuning values
    private double last_kP, last_kI, last_kD;

    // Subsystems
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;

    public Telemetry(ElevatorSubsystem elevator, IntakeSubsystem intake, ClimberSubsystem climber) {
        this.elevator = elevator;
        this.intake = intake;
        this.climber = climber;

        checkElevator();
        checkIntake();
        checkClimber();
        checkTuning(); // Comment out this line if you don’t want live tuning
    }

    // ----------------------
    // Elevator widgets
    // ----------------------
    private void checkElevator() {
        elevatorLayout.addNumber("Encoder", elevator::getEncoder)
                  .withPosition(0, 0)
                  .withSize(1, 1);
    }

    // ----------------------
    // Intake widgets
    // ----------------------
    private void checkIntake() {
        intakeLayout.addNumber("Pneumatic Pressure", intake::getCompressorPressure)
                    .withPosition(0, 0)
                    .withSize(1, 1);
        intakeLayout.addString("Pneumatic State", intake::getPneumaticState)
                    .withPosition(1, 0)
                    .withSize(1, 1);
    }

    // ----------------------
    // Climber widgets
    // ----------------------
    private void checkClimber() {
        climberLayout.addNumber("Motor Speed", climber::getSpeed)
                     .withPosition(0, 0)
                     .withSize(1, 1);
    }

    // ----------------------
    // Optional tuning panel
    // ----------------------
    private void checkTuning() {
        elev_kP = tuningTab.add("Elevator P", elevator.getP()).getEntry();
        elev_kI = tuningTab.add("Elevator I", elevator.getI()).getEntry();
        elev_kD = tuningTab.add("Elevator D", elevator.getD()).getEntry();
    }

    // Only used for live tuning
    public void periodic() {
        // Read-back with defaults so we don’t spam if entries aren’t visible
        double kP = elev_kP != null ? elev_kP.getDouble(elevator.getP()) : elevator.getP();
        double kI = elev_kI != null ? elev_kI.getDouble(elevator.getI()) : elevator.getI();
        double kD = elev_kD != null ? elev_kD.getDouble(elevator.getD()) : elevator.getD();

        // Only update PID controls if they've been changed on shuffleboard
        if (kP != last_kP || kI != last_kI || kD != last_kD) { 
            elevator.setPID(kP, kI, kD); 

            last_kP = kP;
            last_kI = kI;
            last_kD = kD;

            System.out.printf("Updated Elevator PID: P=%.3f I=%.3f D=%.3f%n", kP, kI, kD);
        }
    }
}