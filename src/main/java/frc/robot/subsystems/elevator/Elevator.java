package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    private Arm arm;

    public void setArm(Arm arm) {
        this.arm = arm;
    }

    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
    private final PIDController elevatorController = new PIDController(0.1, 0.0, 0.01);

    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private final GenericEntry positionEntry = tab.add("Elevator Position", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(2, 1)
            .getEntry();
    private final GenericEntry setpointEntry =
            tab.add("Elevator Setpoint", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    private final GenericEntry kPEntry = tab.add("Elevator kP", 0.1).getEntry();
    private final GenericEntry kIEntry = tab.add("Elevator kI", 0.0).getEntry();
    private final GenericEntry kDEntry = tab.add("Elevator kD", 0.01).getEntry();
    private final GenericEntry setpointInput =
            tab.add("Manual Elevator Setpoint", 0.0).getEntry();

    private final GenericEntry canLiftIndicator = tab.add("Can Fold", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "Lime"))
            .withPosition(4, 0)
            .getEntry();

    public Elevator(ElevatorIO io) {
        this.io = io;
        elevatorController.setTolerance(0.01);
        elevatorController.setSetpoint(0.0);

        tab.add("Apply PID", new InstantCommand(this::resetPID, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withProperties(Map.of("Label", "Apply Elevator PID"))
                .withPosition(0, 2);
        tab.add("Move to Manual", new InstantCommand(this::applySetpoint, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withProperties(Map.of("Label", "Move to Setpoint"))
                .withPosition(1, 2);

        tab.add("To L0", new InstantCommand(() -> setPID(0.1, false), this)).withPosition(6, 1);
        tab.add("To L1", new InstantCommand(() -> setPID(0.3, false), this)).withPosition(6, 2);
        tab.add("To L2", new InstantCommand(() -> setPID(0.5, false), this)).withPosition(6, 3);
        tab.add("To L3", new InstantCommand(() -> setPID(0.7, false), this)).withPosition(6, 4);
        tab.add("To L4", new InstantCommand(() -> setPID(0.9, false), this)).withPosition(6, 5);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        positionEntry.setDouble(inputs.position);
        setpointEntry.setDouble(elevatorController.getSetpoint());
        canLiftIndicator.setBoolean(canFold().getAsBoolean());

        if (inputs.limitSwitch && inputs.position != 0) {
            zeroElevator();
        }
    }

    public BooleanSupplier canFold() {
        return () -> arm == null || arm.isInSafeFoldRange();
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void resetPID() {
        elevatorController.setPID(kPEntry.getDouble(0.1), kIEntry.getDouble(0.0), kDEntry.getDouble(0.01));
    }

    public void applySetpoint() {
        setPID(setpointInput.getDouble(0.0), true);
    }

    public void setPID(double setpoint, boolean updateController) {
        if (updateController) {
            elevatorController.setPID(kPEntry.getDouble(0.1), kIEntry.getDouble(0.0), kDEntry.getDouble(0.01));
        }
        elevatorController.setSetpoint(setpoint);
    }

    public void executePID() {
        double output = elevatorController.calculate(inputs.position);
        setVoltage(output);
    }

    public boolean atSetpoint() {
        return elevatorController.atSetpoint();
    }

    public void zeroElevator() {
        io.setPosition(0);
    }

    public Command manualElevator(DoubleSupplier joystick) {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(joystick.getAsDouble() * 6.0),
                interrupted -> setVoltage(0),
                () -> false,
                this);
    }

    public Command elevatorLoad() {
        return moveToPosition(0.1);
    }

    public Command elevatorL1() {
        return moveToPosition(0.3);
    }

    public Command elevatorL2() {
        return moveToPosition(0.5);
    }

    public Command elevatorL3() {
        return moveToPosition(0.7);
    }

    public Command elevatorL4() {
        return moveToPosition(0.9);
    }

    public Command moveToPosition(double position) {
        return new FunctionalCommand(
                () -> setPID(position, true), this::executePID, interrupted -> setVoltage(0), this::atSetpoint, this);
    }
}
