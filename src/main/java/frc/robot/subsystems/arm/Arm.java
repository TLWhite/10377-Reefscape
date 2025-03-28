// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The Arm subsystem controls the robot's arm mechanism. It includes methods for manual control, PID-based control, and
 * predefined positions for the arm.
 */
public class Arm extends SubsystemBase {

    // Motor controller for the arm system
    private final SparkMax armMotor = new SparkMax(ArmConstants.armMotor_ID, SparkMax.MotorType.kBrushless);

    // Absolute encoder for precise arm position tracking
    private final SparkAbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder();

    // Configuration object for the SparkMax motor controller
    private final SparkMaxConfig armSparkMaxConfig = new SparkMaxConfig();

    // Digital input for limit switch to detect arm's home position
    private final DigitalInput limit = new DigitalInput(4);

    // Default arm speed
    double armSpeed = 0.3;

    // Flags for arm safety and folding
    boolean armSafe = false;
    boolean safeFold = false;

    // Motor power variable
    double MotorPower = 0;

    // PID controller for precise arm position control
    private final PIDController armController = new PIDController(3.4, 0, 0);

    // Shuffleboard tab for displaying arm data
    private ShuffleboardTab DS_armTab = Shuffleboard.getTab("arm");

    // Shuffleboard entries for arm position and setpoint
    private GenericEntry DS_armPosition = DS_armTab.add("armValue", armSpeed).getEntry();
    private GenericEntry DS_armSetpoint =
            DS_armTab.add("armSetpoint", armController.getSetpoint()).getEntry();

    /** Constructor for the Arm subsystem. Initializes the motor controller and PID controller. */
    public Arm() {
        configureSparkmax(); // Configure the motor controller
        armController.setTolerance(.0075); // Set PID tolerance
    }

    /** Configures the SparkMax motor controller with specific settings. */
    private void configureSparkmax() {
        // Configure encoder conversion factors for position and velocity
        armSparkMaxConfig
                .encoder
                .positionConversionFactor(1) // Conversion factor for position
                .velocityConversionFactor(1); // Conversion factor for velocity

        // Invert the absolute encoder
        armSparkMaxConfig.absoluteEncoder.inverted(true);

        // Apply the configuration to the motor controller
        armMotor.configure(armSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the power for the arm motor.
     *
     * @param motor The motor to control
     * @param power The power level to set (-1.0 to 1.0)
     */
    public void setarmPower(SparkMax motor, double power) {
        motor.set(power);
    }

    /**
     * Sets the arm motor power with limits to prevent over-travel.
     *
     * @param power The desired power level
     */
    public void setarmMotor(double power) {
        double output = 0;
        double limitedSpeed = armLimit(power);

        // Adjust motor output based on arm position
        if (getarmPosition() >= .25) {
            output = Math.sin(getarmPosition() * 2 * Math.PI) * .02 + limitedSpeed;
        } else {
            output = Math.sin(getarmPosition() * 2 * Math.PI) * .02125 + limitedSpeed;
        }

        // Display debug information on SmartDashboard
        SmartDashboard.putNumber("arm axis", power);
        SmartDashboard.putNumber("Speed", Math.sin(getarmPosition() * 2 * Math.PI) * .06 + power);

        // Set the motor power with limits
        setarmPower(armMotor, armLimit(output));
    }

    /** Stops the arm motor by setting its power to zero. */
    public void stoparmMotor() {
        this.setarmMotor(0);
    }

    /**
     * Limits the arm motor power based on the current arm position to avoid damaging the arm.
     *
     * @param speed The desired motor speed
     * @return The limited motor speed
     */
    private double armLimit(double speed) {
        double output = speed;

        // Limit motor speed based on encoder position
        if ((armAbsEncoder.getPosition() < 0.7 && armAbsEncoder.getPosition() > .315) && speed > 0) {
            output = 0;
        } else if ((armAbsEncoder.getPosition() >= 0.7 || armAbsEncoder.getPosition() < .109) && speed < 0) {
            output = 0;
        }
        return output;
    }

    // Trigger to limit arm movement if the arm is too low (below position 0.09)
    public Trigger armLimiter() {
        return new Trigger(() -> true); // Always returns true (placeholder logic).
        // This could be updated to activate when the arm position is below 0.09.
    }

    // Trigger to activate intake if the arm is at L1 (above or equal to position 0.42)
    public Trigger armIntake() {
        return new Trigger(() -> this.getarmPosition() >= .42);
        // Activates when the arm position is above or equal to 0.42.
    }

    // Method to retrieve the current arm position from the encoder
    public double getarmPosition() {
        return (armAbsEncoder.getPosition());
        // Returns the current arm position as a double value from the encoder.
    }

    // Method to set the desired target arm position for PID control
    public void setarmPID(double setPoint) {
        this.armController.setSetpoint(setPoint);
        // Sets the desired position (setpoint) for the arm using the PID controller.
    }

    // Method to execute PID control to adjust the motor power and move the arm
    // towards the target position
    public void executearmPID() {
        setarmMotor((this.armController.calculate(this.getarmPosition())));
        // Calculates the appropriate motor power based on the PID controller and applies it to the arm motor.
    }

    // Method to check if the arm has reached its target position based on the PID tolerance
    public boolean armAtSetpoint() {
        return this.armController.atSetpoint();
        // Returns true if the arm has reached its target position within the specified tolerance.
    }

    // Command to run the arm motor with PID control
    public Command armPIDCommandDefault(BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {
                    this.safeFold = canFold.getAsBoolean(); // Initialize: Update the safeFold flag.
                },
                () -> {
                    this.executearmPID(); // Execute: Run the PID control to adjust arm position.
                    this.safeFold = canFold.getAsBoolean(); // Update the safeFold flag.
                },
                interrupted -> {
                    // No specific action when interrupted.
                },
                () -> false, // Finish condition: Always false, meaning the command never finishes automatically.
                this); // Subsystem: This command is bound to the arm subsystem.
    }

    // Command for manual control of the arm motor, using joystick input for arm movement
    public Command Manualarm(DoubleSupplier armJoystick, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {
                    this.safeFold = canFold.getAsBoolean(); // Initialize: Update the safeFold flag.
                },
                () -> {
                    this.setarmMotor(
                            armJoystick.getAsDouble() * 0.3); // Execute: Set arm motor power based on joystick input.
                },
                interrupted -> {
                    this.setarmMotor(0); // Stop the motor if interrupted.
                    this.setarmPID(getarmPosition()); // Reset to the current arm position.
                },
                () -> false, // Finish condition: Always false, meaning the command never finishes automatically.
                this); // Subsystem: This command is bound to the arm subsystem.
    }

    // Command to home the arm using a limit switch
    public Command homeCommand() {
        return new FunctionalCommand(
                () -> {
                    // Initialize: No specific action needed.
                },
                () -> {
                    setarmPower(armMotor, -0.1); // Apply power to move the arm towards the home position.
                },
                interrupted -> {
                    setarmPower(armMotor, 0); // Stop the motor if interrupted.
                    this.setarmPID(getarmPosition()); // Reset to the current arm position.
                    SmartDashboard.putString("complete", "complete"); // Display completion status.
                },
                () -> !limit.get(), // Finish condition: Stops when the limit switch is triggered.
                this); // Subsystem: This command is bound to the arm subsystem.
    }

    // Command to raise the arm manually
    public Command raisearm() {
        return new FunctionalCommand(
                () -> {
                    // Initialize: No specific action needed.
                },
                () -> {
                    this.setarmMotor(0.3); // Apply power to raise the arm.
                },
                interrupted -> {
                    // No specific action when interrupted.
                },
                () -> false, // Finish condition: Always false, meaning the command never finishes automatically.
                this); // Subsystem: This command is bound to the arm subsystem.
    }

    // Factory method to create a command for moving the arm to a specific position
    public Command armCommandFactory(BooleanSupplier canFold, double position) {
        return new FunctionalCommand(
                () -> {
                    this.setarmPID(position); // Set the target position for the arm.
                    this.safeFold = canFold.getAsBoolean(); // Update the safeFold flag.
                },
                () -> {
                    this.safeFold = canFold.getAsBoolean(); // Update the safeFold flag.
                    this.executearmPID(); // Execute PID control to move the arm.
                },
                interrupted -> {
                    this.setarmPID(this.getarmPosition()); // Reset to the current arm position if interrupted.
                    this.setarmMotor(0); // Stop the motor.
                },
                () -> this.armAtSetpoint(), // Finish condition: Check if the arm has reached the target position.
                this); // Subsystem: This command is bound to the arm subsystem.
    }

    // Command to hold the arm at its current position
    public Command startarmCommand() {
        return this.runOnce(() -> {
            this.setarmPID(this.getarmPosition()); // Set the PID setpoint to the current position.
        });
    }

    // Command to move the arm to a safe position
    public Command armSafety(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.25); // Move the arm to a predefined safe position.
    }

    // Command to move the arm to position L0
    public Command armL0(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.2);
    }

    // Command to move the arm to the algae intake position
    public Command armAlgeaIntake(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.13);
    }

    // Commands for moving the arm to various predefined positions (L1, L2, L3, L4)
    public Command armL1(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.25); // Move the arm to position L1.
    }

    public Command armL2(BooleanSupplier canFold) {
        return armCommandFactory(canFold, .179); // Move the arm to position L2.
    }

    public Command armL3(BooleanSupplier canFold) {
        return armCommandFactory(canFold, .1365); // Move the arm to position L3.
    }

    public Command armL4(BooleanSupplier canFold) {
        return armCommandFactory(canFold, .0887); // Move the arm to position L4.
    }

    // Command to move the arm to position A1
    public Command armA1(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.2);
    }

    // Command to move the arm to position A2
    public Command armA2(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.210);
    }

    // Command to move the arm to the barge position
    public Command armBarge(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.289); // Move the arm to the barge position.
    }

    // Command to move the arm to the processor position
    public Command armProcessor(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.2);
    }

    // Command to move the arm to the climber position
    public Command armClimber(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.4);
    }

    // Command to exit the current arm state, keeping it in its current position
    public Command ExitState(BooleanSupplier canFold) {
        return armCommandFactory(canFold, this.getarmPosition());
    }

    // Periodic method called once per scheduler run, used for updating real-time data
    @Override
    public void periodic() {
        this.DS_armPosition.setDouble(getarmPosition()); // Update the arm's current position on Shuffleboard.
        this.DS_armSetpoint.setDouble(armController.getSetpoint()); // Update the PID setpoint on Shuffleboard.
        SmartDashboard.putBoolean("arm limit", limit.get()); // Display the limit switch status on SmartDashboard.
    }
}
