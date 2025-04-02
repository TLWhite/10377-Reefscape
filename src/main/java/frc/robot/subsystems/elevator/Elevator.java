package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController; // For controlling the elevator's position using PID
import edu.wpi.first.networktables.GenericEntry; // For Shuffleboard entries
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; // For Shuffleboard widget types
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // For creating Shuffleboard tabs
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // For managing Shuffleboard tabs
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands
import edu.wpi.first.wpilibj2.command.FunctionalCommand; // For creating functional commands
import edu.wpi.first.wpilibj2.command.InstantCommand; // For creating instant commands
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Base class for subsystems
import frc.robot.subsystems.arm.Arm; // For interacting with the Arm subsystem
import java.util.Map; // For creating property maps
import java.util.function.BooleanSupplier; // Functional interface for boolean values
import java.util.function.DoubleSupplier; // Functional interface for double values

/**
 * The Elevator subsystem controls the robot's elevator mechanism. It uses PID control for precise positioning and
 * integrates with Shuffleboard for monitoring and control.
 */
public class Elevator extends SubsystemBase {
    // Reference to the Arm subsystem for checking if folding is safe
    private Arm arm;

    /**
     * Sets the Arm subsystem reference.
     *
     * @param arm The Arm subsystem instance.
     */
    public void setArm(Arm arm) {
        this.arm = arm;
    }

    // Hardware interface for the elevator
    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    // PID controller for controlling the elevator's position
    private final PIDController elevatorController = new PIDController(0.1, 0.0, 0.01);

    // Shuffleboard tab for the elevator subsystem
    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    // Shuffleboard entries for monitoring and controlling the elevator
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

    private final GenericEntry canLiftIndicator = tab.add(
                    "Can Fold", true) // TODO: Changed this value to true to override
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "Lime"))
            .withPosition(4, 0)
            .getEntry();

    /**
     * Constructor for the Elevator subsystem.
     *
     * @param io The hardware interface for the elevator.
     */
    public Elevator(ElevatorIO io) {
        this.io = io;
        elevatorController.setTolerance(0.01); // Set the tolerance for the PID controller
        elevatorController.setSetpoint(0.0); // Initialize the setpoint to 0.0

        // Add Shuffleboard controls for applying PID values and moving the elevator
        tab.add("Apply PID", new InstantCommand(this::resetPID, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withProperties(Map.of("Label", "Apply Elevator PID"))
                .withPosition(0, 2);
        tab.add("Move to Manual", new InstantCommand(this::applySetpoint, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withProperties(Map.of("Label", "Move to Setpoint"))
                .withPosition(1, 2);

        // Add Shuffleboard buttons for moving the elevator to predefined positions
        tab.add("To L0", new InstantCommand(() -> setPID(0.1, false), this)).withPosition(6, 1);
        tab.add("To L1", new InstantCommand(() -> setPID(0.3, false), this)).withPosition(6, 2);
        tab.add("To L2", new InstantCommand(() -> setPID(0.5, false), this)).withPosition(6, 3);
        tab.add("To L3", new InstantCommand(() -> setPID(0.7, false), this)).withPosition(6, 4);
        tab.add("To L4", new InstantCommand(() -> setPID(0.9, false), this)).withPosition(6, 5);
    }

    /** Periodic method called by the scheduler. Updates sensor inputs and Shuffleboard entries. */
    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update the input data from the hardware
        positionEntry.setDouble(inputs.position); // Update the position on Shuffleboard
        setpointEntry.setDouble(elevatorController.getSetpoint()); // Update the setpoint on Shuffleboard
        canLiftIndicator.setBoolean(canFold().getAsBoolean()); // Update the "Can Fold" indicator

        // Zero the elevator if the limit switch is triggered and the position is not already zero
        if (inputs.limitSwitch && inputs.position != 0) {
            zeroElevator();
        }
    }

    /**
     * Checks if the elevator can fold safely.
     *
     * @return A BooleanSupplier indicating if folding is safe.
     */
    public BooleanSupplier canFold() {
        return () -> arm == null || arm.isInSafeFoldRange();
    }

    /**
     * Determines whether the elevator can fold safely.
     *
     * @return A BooleanSupplier that returns true if folding is allowed, false otherwise. - Returns true if the Arm
     *     subsystem is not set (arm == null). - Returns true if the Arm subsystem is set and the arm is within the safe
     *     folding range. TODO: Set the sa // If the arm subsystem is null, folding is considered safe. // Otherwise, it
     *     checks if the arm is within the safe folding range using the isInSafeFoldRange() method.
     *     <p>/** Sets the voltage to be applied to the elevator motor.
     * @param volts The voltage to apply.
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Resets the PID controller with updated values from Shuffleboard. */
    public void resetPID() {
        elevatorController.setPID(kPEntry.getDouble(0.1), kIEntry.getDouble(0.0), kDEntry.getDouble(0.01));
    }

    /** Applies the setpoint from Shuffleboard to the PID controller. */
    public void applySetpoint() {
        setPID(setpointInput.getDouble(0.0), true);
    }

    /**
     * Sets the PID controller's target setpoint.
     *
     * @param setpoint The target setpoint.
     * @param updateController Whether to update the PID controller values.
     */
    public void setPID(double setpoint, boolean updateController) {
        if (updateController) {
            elevatorController.setPID(kPEntry.getDouble(0.1), kIEntry.getDouble(0.0), kDEntry.getDouble(0.01));
        }
        elevatorController.setSetpoint(setpoint);
    }

    /** Executes the PID controller and sets the motor voltage. */
    public void executePID() {
        double output = elevatorController.calculate(inputs.position);
        setVoltage(output);
    }

    /**
     * Checks if the elevator is at the target setpoint.
     *
     * @return True if at the setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        return elevatorController.atSetpoint();
    }

    /** Zeros the elevator's position. */
    public void zeroElevator() {
        io.setPosition(0);
    }

    /**
     * Creates a manual control command for the elevator.
     *
     * @param joystick The joystick input.
     * @return The manual control command.
     */
    public Command manualElevator(DoubleSupplier joystick) {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(joystick.getAsDouble() * 6.0),
                interrupted -> setVoltage(0),
                () -> false,
                this);
    }

    // Commands for moving the elevator to predefined positions
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

    /**
     * Creates a command to move the elevator to a specific position.
     *
     * @param position The target position.
     * @return The command to move the elevator.
     */
    public Command moveToPosition(double position) {
        return new FunctionalCommand(
                () -> setPID(position, true), this::executePID, interrupted -> setVoltage(0), this::atSetpoint, this);
    }
}

/* GitHub Copilot
Here is the Elevator.java file with detailed comments added to explain each part of the code:

Explanation of the Code:

1. Subsystem Overview:

    The Elevator class represents the elevator subsystem, which controls the vertical movement of the robot's elevator mechanism.
    It uses a PID controller for precise positioning and integrates with Shuffleboard for monitoring and control.

2. Key Features:

    PID Control: The elevatorController is used to control the elevator's position.
    Shuffleboard Integration: Provides real-time monitoring and control of the elevator's state and setpoints.
    Predefined Positions: Commands are provided to move the elevator to predefined positions (e.g., L0, L1, etc.).

3. Commands:

    Manual Control: Allows the elevator to be controlled manually using a joystick.
    Predefined Position Commands: Moves the elevator to specific positions using PID control.

4. Safety Features:

    The canFold method ensures the elevator can fold safely by checking the state of the Arm subsystem.
    The zeroElevator method resets the elevator's position when the limit switch is triggered. */
