package frc.robot.subsystems.arm;

// Importing necessary libraries and classes
import edu.wpi.first.math.controller.PIDController; // For PID control of the arm
import edu.wpi.first.networktables.GenericEntry; // For Shuffleboard entries
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; // For Shuffleboard widget types
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // For creating Shuffleboard tabs
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // For managing Shuffleboard tabs
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // For displaying data on the SmartDashboard
import edu.wpi.first.wpilibj2.command.Command; // Base class for commands
import edu.wpi.first.wpilibj2.command.FunctionalCommand; // For creating functional commands
import edu.wpi.first.wpilibj2.command.InstantCommand; // For creating instant commands
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Base class for subsystems
import java.util.Map; // For creating property maps
import java.util.function.BooleanSupplier; // Functional interface for boolean values
import java.util.function.DoubleSupplier; // Functional interface for double values

/**
 * The Arm subsystem controls the robot's arm mechanism. It uses PID control for precise positioning and integrates with
 * Shuffleboard for monitoring and control.
 */
public class Arm extends SubsystemBase {
    // Hardware interface for the arm
    private final ArmIO io;
    private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

    // PID controller for controlling the arm's position
    private final PIDController armController = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);

    // Tracks whether the arm has been initialized
    private boolean initialized = false;

    // Shuffleboard tab for the arm subsystem
    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    // Shuffleboard entries for monitoring and controlling the arm
    private final GenericEntry positionEntry = tab.add("Arm Position", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 2)
            .withPosition(6, 4)
            .getEntry();
    private final GenericEntry setpointEntry = tab.add("Target Setpoint", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .getEntry();
    private final GenericEntry kPEntry =
            tab.add("kP", ArmConstants.P).withPosition(4, 0).getEntry();
    private final GenericEntry kIEntry =
            tab.add("kI", ArmConstants.I).withPosition(4, 1).getEntry();
    private final GenericEntry kDEntry =
            tab.add("kD", ArmConstants.D).withPosition(4, 2).getEntry();
    private final GenericEntry inputSetpoint =
            tab.add("Manual Setpoint Input", 0.0).withPosition(0, 1).getEntry();
    private final GenericEntry canLiftIndicator = tab.add("Can Lift", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "Lime"))
            .withPosition(6, 1)
            .getEntry();
    private final GenericEntry encoderPosition =
            tab.add("Abs Encoder Pos", 0.0).withPosition(0, 5).getEntry();
    private final GenericEntry encoderOffset =
            tab.add("Encoder Offset", 0.0).withPosition(0, 6).getEntry();
    private final GenericEntry armAngleIndicator =
            tab.add("Arm Angle (deg)", 0.0).withPosition(0, 7).getEntry();
    private final GenericEntry l4Entry =
            tab.add("Setpoint L4", ArmConstants.L4_POS).withPosition(0, 2).getEntry();
    private final GenericEntry l3Entry =
            tab.add("Setpoint L3", ArmConstants.L3_POS).withPosition(0, 3).getEntry();
    private final GenericEntry l2Entry =
            tab.add("Setpoint L2", ArmConstants.L2_POS).withPosition(0, 4).getEntry();
    private final GenericEntry l1Entry =
            tab.add("Setpoint L1", ArmConstants.L1_POS).withPosition(0, 5).getEntry();
    private final GenericEntry l0Entry =
            tab.add("Setpoint L0", ArmConstants.L0_POS).withPosition(0, 6).getEntry();
    private final GenericEntry loadEntry =
            tab.add("Setpoint Load", ArmConstants.LOAD_POS).withPosition(0, 7).getEntry();

    /**
     * Constructor for the Arm subsystem.
     *
     * @param io The hardware interface for the arm.
     */
    public Arm(ArmIO io) {
        this.io = io;
        armController.setTolerance(ArmConstants.PID_TOLERANCE);
        armController.setSetpoint(0.0);

        // Add Shuffleboard controls for updating PID values and moving the arm
        tab.add("Update PID", new InstantCommand(this::resetPID, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 0)
                .withProperties(Map.of("Label", "Apply PID Values"));
        tab.add("Move to Manual", new InstantCommand(this::applySetpoint, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 1)
                .withProperties(Map.of("Label", "Move to Manual Setpoint"));
        tab.add("Move to L4", new InstantCommand(() -> setPID(l4Entry.getDouble(ArmConstants.L4_POS)), this))
                .withPosition(1, 2);
        tab.add("Move to L3", new InstantCommand(() -> setPID(l3Entry.getDouble(ArmConstants.L3_POS)), this))
                .withPosition(1, 3);
        tab.add("Move to L2", new InstantCommand(() -> setPID(l2Entry.getDouble(ArmConstants.L2_POS)), this))
                .withPosition(1, 4);
        tab.add("Move to L1", new InstantCommand(() -> setPID(l1Entry.getDouble(ArmConstants.L1_POS)), this))
                .withPosition(1, 5);
        tab.add("Move to L0", new InstantCommand(() -> setPID(l0Entry.getDouble(ArmConstants.L0_POS)), this))
                .withPosition(1, 6);
        tab.add("Move to Load", new InstantCommand(() -> setPID(loadEntry.getDouble(ArmConstants.LOAD_POS)), this))
                .withPosition(1, 7);
    }

    /** Periodic method called by the scheduler. Updates sensor inputs and Shuffleboard entries. */
    @Override
    public void periodic() {
        if (!initialized) {
            setPID(0.0);
            initialized = true;
        }

        io.updateInputs(inputs);
        positionEntry.setDouble(inputs.position);
        setpointEntry.setDouble(armController.getSetpoint());
        canLiftIndicator.setBoolean(isInSafeFoldRange());
        encoderPosition.setDouble(inputs.position);
        encoderOffset.setDouble(inputs.offset);
        armAngleIndicator.setDouble((inputs.position - inputs.offset) * 360);
        SmartDashboard.putBoolean("arm limit", inputs.limitSwitch);
    }

    /**
     * Sets the motor voltage with a feedforward term.
     *
     * @param volts The voltage to set.
     */
    public void setVoltage(double volts) {
        double kfVolts = Math.sin(inputs.position * 2 * Math.PI) * ArmConstants.KF_COEFFICIENT * 12.0;
        io.setVoltage(volts + kfVolts);
    }

    /** Stops the motor by setting the voltage to zero. */
    public void stopMotor() {
        setVoltage(0.0);
    }

    /** Resets the PID controller with updated values from Shuffleboard. */
    public void resetPID() {
        armController.setPID(
                kPEntry.getDouble(ArmConstants.P),
                kIEntry.getDouble(ArmConstants.I),
                kDEntry.getDouble(ArmConstants.D));
    }

    /** Applies the setpoint from Shuffleboard to the PID controller. */
    public void applySetpoint() {
        setPID(inputSetpoint.getDouble(0.0));
    }

    /**
     * Sets the PID controller's target setpoint.
     *
     * @param setpoint The target setpoint.
     */
    public void setPID(double setpoint) {
        armController.setSetpoint(setpoint);
    }

    /** Executes the PID controller and sets the motor voltage. */
    public void executePID() {
        double output = armController.calculate(inputs.position);
        setVoltage(output);
    }

    /**
     * Checks if the arm is at the target setpoint.
     *
     * @return True if at the setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        return armController.atSetpoint();
    }

    /**
     * Gets the current position of the arm.
     *
     * @return The arm's position.
     */
    public double getPosition() {
        return inputs.position;
    }

    /**
     * Checks if the arm is in a safe range for folding.
     *
     * @return True if in the safe range, false otherwise.
     */
    public boolean isInSafeFoldRange() {
        return inputs.position >= ArmConstants.MIN_SAFE_FOLD_POS && inputs.position <= ArmConstants.MAX_SAFE_FOLD_POS;
    }

    // Commands for moving the arm to specific positions
    public Command armL0(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L0_POS, canFold);
    }

    public Command armL1(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L1_POS, canFold);
    }

    public Command armL2(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L2_POS, canFold);
    }

    public Command armL3(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L3_POS, canFold);
    }

    public Command armL4(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L4_POS, canFold);
    }

    public Command armLoad(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.LOAD_POS, canFold);
    }

    /**
     * Creates a manual control command for the arm.
     *
     * @param joystick The joystick input.
     * @param canFold A supplier indicating if folding is allowed.
     * @return The manual control command.
     */
    /*     public Command manualArm(DoubleSupplier joystick, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(canFold.getAsBoolean() ? joystick.getAsDouble() * 3.0 : 0.0),
                interrupted -> stopMotor(),
                () -> false,
                this);
    } */

    /**
     * Creates a manual control command for the arm.
     *
     * @param joystick A DoubleSupplier providing the joystick input for controlling the arm. The joystick value
     *     determines the speed and direction of the arm's movement.
     * @param canFold A BooleanSupplier indicating whether folding is allowed. If folding is not allowed, the arm will
     *     not move.
     * @return A FunctionalCommand that allows manual control of the arm.
     */
    public Command manualArm(DoubleSupplier joystick, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {}, // Initialization logic: No initialization is required for this command.
                () -> {
                    // Execution logic: Set the motor voltage based on joystick input.
                    // If folding is allowed (canFold is true), apply voltage proportional to the joystick value.
                    // Otherwise, set the voltage to 0 to prevent movement.
                    setVoltage(canFold.getAsBoolean() ? joystick.getAsDouble() * 3.0 : 0.0);
                },
                interrupted -> {
                    // Interruption logic: Stop the motor when the command is interrupted.
                    stopMotor();
                },
                () -> false, // Completion logic: This command never completes on its own.
                this // The subsystem this command requires (the Arm subsystem).
                );
    }

    /**
     * Creates a command to move the arm to a target position.
     *
     * @param target The target position.
     * @param canFold A supplier indicating if folding is allowed.
     * @return The command to move the arm.
     */
    private Command createArmCommand(double target, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> setPID(target),
                () -> {
                    if (canFold.getAsBoolean()) executePID();
                    else stopMotor();
                },
                interrupted -> stopMotor(),
                this::atSetpoint,
                this);
    }
}

/*

Explanation of Comments:

1.PID Constants:

        These constants are used to configure the PID controller for precise arm positioning.
        P, I, and D are the proportional, integral, and derivative gains, respectively.
        PID_TOLERANCE defines how close the arm must be to the target position to be considered "at setpoint."

2. Setpoints:
        These constants define predefined positions for the arm (e.g., L0_POS for Level 0, L1_POS for Level 1).
        These positions are used in commands to move the arm to specific levels.

3.Safe Folding Range:
        The MIN_SAFE_FOLD_POS and MAX_SAFE_FOLD_POS define the range within which the arm can safely fold without causing damage.

4.Physical Constraints:

        MIN_POS and MAX_POS define the physical limits of the arm's movement.

5.Feedforward Coefficient:

        KF_COEFFICIENT is used to calculate a feedforward voltage to counteract gravity.

6.Hardware IDs:

        These constants define the IDs and ports for the arm's motor and limit switch.

7. Encoder Configuration:

        ENCODER_OFFSET is used to adjust the encoder's zero position.
        ENCODER_PORT specifies the port where the encoder is connected.

8. Simulation and Physical Properties:

        GEAR_RATIO defines the gear ratio of the arm mechanism.
        ARM_MASS and ARM_LENGTH are used for simulation and physics calculations. */
