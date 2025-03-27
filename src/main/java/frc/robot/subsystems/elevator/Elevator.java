// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

// Import required classes
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator subsystem. This is where the elevator motor control and logic reside. */

    // Define the SparkMax motor controllers for both sides of the elevator (left and
    // right).
    private final SparkMax leftMotor = new SparkMax(
            ElevatorConstants.leftMotor_ID,
            SparkMax.MotorType.kBrushless); // Replace '16' with the actual CAN ID for the left motor

    private final SparkMax rightMotor = new SparkMax(
            ElevatorConstants.rightMotor_ID,
            SparkMax.MotorType.kBrushless); // Replace '15' with the actual CAN ID for the right motor

    private final SparkMax armMotor = new SparkMax(ArmConstants.armMotor_ID, SparkMax.MotorType.kBrushless);
    private final SparkMaxConfig leftElevatorConfig =
            new SparkMaxConfig(); // Create a configuration object for the motor
    private final SparkMaxConfig rightElevatorConfig =
            new SparkMaxConfig(); // Create a configuration object for the motor

    public static final int L1 = 5; // TODO set this value
    public static final int L2 = 10; //  TODO set this value
    public static final int L3 = 15; // TODO set this value
    public static final int L4 = 20; // TODO set this value

    public static final int downPos = 0;
    public static final int bottomPos = 0;

    // Flag to check if the elevator is allowed to lift based on a condition.

    // Create a PID controller to control the elevator position, with initial PID
    // values (Proportional, Integral, Derivative).

    private final PIDController elevatorController = new PIDController(0.08, 0, 0); // TODO tune PID values

    // Setup Shuffleboard (WPILib's dashboard) for real-time monitoring of the
    // elevator state during the match.

    private ShuffleboardTab DS_ElevatorTab = Shuffleboard.getTab("Elevator");
    private GenericEntry DS_ElevatorPosition =
            DS_ElevatorTab.add("ElevatorValue", 0).getEntry(); // Entry for elevator
    // position

    private GenericEntry DS_canLift = DS_ElevatorTab.add("CanLift", true).getEntry(); // Entry for canLift

    private GenericEntry DS_ElevatorSetpoint =
            DS_ElevatorTab.add("Setpoint", elevatorController.getSetpoint()).getEntry();

    // Default max elevator speed as defined by Shuffleboard.
    // double maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

    // Default max elevator speed as defined by Shuffleboard.
    // double maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

    public Elevator() {
        // Set both motor controllers to Coast mode to stop the motors from coasting
        // when no power is applied.
        configureSparkmax();

        // Set initial PID controller setpoint to current elevator position.
        elevatorController.setSetpoint(getPosition());
        elevatorController.setTolerance(1); // Set tolerance to 1 (tolerance defines when the PID controller considers
        // the
        // target reached)
        // rightMotor.setPosition(0); // Possible initial position setting for
        // the second motor (commented out)

    private void configureSparkmax() {
        // Set the idle mode to 'brake', ensuring the motor resists movement when not powered.
        // This helps prevent the elevator from moving unintentionally due to gravity or
        // momentum.
        leftElevatorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMITS)
                .inverted(true); // Protects motor hardware by limiting current. TODO check if this is correct
        leftElevatorConfig
                .softLimit
                .forwardSoftLimit(ElevatorConstants.maxPos) // TODO set this value
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(ElevatorConstants.minPos) // TODO set this value
                .reverseSoftLimitEnabled(false);

        rightElevatorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMITS)
                .inverted(true); // Protects motor hardware by limiting current.TODO check if this is correct
        rightElevatorConfig
                .softLimit
                .forwardSoftLimit(ElevatorConstants.maxPos) // TODO set this value
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(ElevatorConstants.minPos) // TODO set this value
                .reverseSoftLimitEnabled(false);

        // Apply the configurations to the motor controllers.
        leftMotor.configure(leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // Method to check if the elevator has reached its setpoint.
    public boolean elevatorAtSetpoint() {
        return this.elevatorController.atSetpoint(); // Returns true if the elevator is at its setpoint.
    }

    public double getElevatorPosition() {
        return (elevatorTalonStrb.getPosition().getValueAsDouble());
    }

    /*
     * Setters for the elevator motors.
     */

    // Helper method to set the power (speed) of a given motor.
    private void setMotorPower(SparkMax motor, double power) {
        motor.set(power);
        // Apply the desired power to the motor
    }

    // Method to set power to both elevator motors, considering limits.
    public void setElevatorMotor(double power) {
        double output = elevatorLimit(power);
        double kFValue = kfValueSetter();
        SmartDashboard.putNumber("LimitOutput", output); //
        SmartDashboard.putNumber("Kf value", kFValue);
        // Apply limit on power to prevent the elevator from exceeding boundaries (e.g.,
        // going beyond the upper or lower limit).
        setMotorPower(leftMotor, output + kFValue); // Apply power to the left motor with limits.
        setMotorPower(rightMotor, output + kFValue); // Apply power to the right motor with limits.
        // we added kF value to here to make sure the elevator will hold when it is fed
        // a zero from its limiter

    }

    public boolean atTopLimint() {
        return getPosition() >= 218;
    }

    // Limit the motor power based on certain conditions such as the current
    // position of the elevator and whether it can lift.
    private double elevatorLimit(double power) {
        double output = 0; // Default output is 0 (no power).
        SmartDashboard.putNumber("Input", power);
        // If the elevator can't lift or if the elevator is at the top or bottom, set
        // the output power to a small value.
        if ((getPosition() >= ElevatorConstants.maxPos && power > 0) // Positive Power makes the

                // robot go up negative makes the robot go down
                || (getPosition() <= ElevatorConstants.minPos && power < 0)) {

            output = 0.0;
            // Output is zero but is given a kf value of .02 when it is applied to the motor
            // to allow the elevator to hold.
        } else {
            output = power;
            // Otherwise, apply the requested power to the motors.
        }
        SmartDashboard.putNumber("output", output);
        return output; // Return the calculated output power.
    }

    // Methods for controlling the elevator using PID.
    public void setPID(double setPoint) {

        this.elevatorController.setSetpoint(setPoint); // Tells the PID controller what the setpoint is
    }

    public double kfValueSetter() {
        double kF = 0;
        if (this.getPosition() < 0.5) {
            kF = 0.01; // VALUE FOR BELLOW A POSTION TO ALLOW ELEVAT0R TO ZERO TODO SET VALUE
        } else {
            kF = 0.025; // TODO SET VALUE
        }

        return kF;
    }

    public double Throttle() {
        return ((1 - 0.5 * (this.getPosition() / 65.71))); // This method is used to slow the drivespeed down based
        // on
        // the elevator position
        // any slower than this will make slowmode not be able to be held down on the
        // controller or the robot cannot move (OG value for division problem 115 new
        // value is 65.71)
    }

    public Trigger canFold() { // this method sets whether the arm can fold back based on the elevator
        // position this prevents folding back into the crossmembers
        // Original values in order 26, 50.5, 59, 104
        // new values in order 14.86, 28.86, 33.71, 59.43
        return new Trigger(
                () -> true); // (!((this.getElevatorPosition() >= 14.86) && (this.getElevatorPosition() <= 28.86))
        // || !((this.getElevatorPosition() >= 33.71) && (this.getElevatorPosition() <= 59.43))));

    }

    //

    public void zeroElevator() // Check if the built-in reverse limit
            {
        {
            //  encoder.setPosition(0); // Reset the value to zero//TODO: Need to update with encoder
        }
    }

    // Method to calculate and apply the PID output to move the elevator towards the
    // setpoint.
    public void executeElevatorPID() {
        // Call the PID controller to calculate the required power and apply it to the
        // motors.
        double PIDValue = this.elevatorController.calculate(this.getPosition());

        setElevatorMotor(PIDValue);
    }

    // Command for manual control of the elevator during teleop, allowing the driver
    // to move the elevator with a joystick.
    public Command ManualElevator(DoubleSupplier elevatorJoystick) {
        return new FunctionalCommand(
                () -> {
                    // Update whether lifting is allowed based on arm limiter.
                },
                () -> {
                    // Use joystick input to control elevator power. Apply a scaling factor of 0.2
                    // for smooth control.
                    this.setElevatorMotor(elevatorJoystick.getAsDouble() * 0.8); // 0.4 + 0.1
                    // Update lifting condition.
                },
                interrupted -> {
                    this.setPID(this.getPosition());
                }, // If interrupted, reset the PID setpoint.
                () -> {
                    return false; // No specific condition is required for the command to finish.
                },
                this);
    }

    // A command for starting the elevator with no movement but initializing
    // necessary states.
    public Command startCommand() {
        return this.runOnce(
                () -> {
                    // Update whether lifting is allowed.
                    this.setPID(this.getPosition());
                } // Set the PID setpoint to current position.
                );
    }

    // Default PID elevator control command, continuously adjusting the elevator's
    // position based on the PID controller.
    public Command elevatorPIDCommandDefault(BooleanSupplier armLimiter) {
        return new FunctionalCommand(
                () -> {
                    // Update lifting condition.
                },
                () -> {
                    this.executeElevatorPID(); // Execute PID control to adjust the elevator's position.
                },
                interrupted -> {
                    // Update lifting condition when interrupted.
                },
                () -> {
                    return false; // No condition for command termination.
                },
                this);
    }

    // Command to move the elevator to a position based on a given setpoint. 1.09
    // rotations = 1 inch of height
    public Command MovetoPosition(double position) {
        return new FunctionalCommand(
                () -> {
                    // Update lifting condition.
                    this.setPID(position); // Set PID setpoint to 0 (L1 position).
                },
                () -> {
                    this.executeElevatorPID(); // Execute PID control to move the elevator.
                },
                interrupted -> {
                    this.setPID(this.getPosition());

                    this.setElevatorMotor(0);
                }, // Nothing new runs when interrupted
                () -> (this.elevatorAtSetpoint()), // Check if the elevator has reached L1.
                this);
    }

    public Command ElevatorLoad() {

        return MovetoPosition(
                0); // TODO set this value by manually moving the elevator to the to position and reading the value
    }

    // Command to move the elevator to the L1 position (0).
    public Command ElevatorL1() {

        return MovetoPosition(
                10.88); // TODO set this value by manually moving the elevator to the to position and reading the value
    }

    // Command to move the elevator to the L2 position
    public Command ElevatorL2() {

        return MovetoPosition(
                66.43); // TODO set this value by manually moving the elevator to the to position and reading the value
    }

    // Command to move the elevator to the L3 position
    public Command ElevatorL3() {

        return MovetoPosition(
                133.998); // TODO set this value by manually moving the elevator to the to position and reading the
        // value
    }

    // Command to move the elevator to the L4 position OG (113.7). New(64.97)
    public Command ElevatorL4() {

        return MovetoPosition(
                218); // TODO set this value by manually moving the elevator to the top and reading the value
    }

    // Command to exit the current elevator state and maintain its position.
    public Command ExitState() {
        return MovetoPosition(this.getPosition());
    }

    // Periodic method called once per scheduler run to update real-time data on
    // Shuffleboard for monitoring.
    @Override
    public void periodic() {

        // checkLimitAndReset();

        // Update the maximum speed value from Shuffleboard.
        // TLWhite Limit switched commeted out, due brok
        // this.maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

        // Update the current position of the elevator.
        /*    this.DS_ElevatorPosition.setDouble(getPosition());
        if ((!lowerLimitSwitchPort.get()) && (this.getPosition() != 0)) {
            this.zeroElevator();
            SmartDashboard.getNumber("leftMotor speed", leftMotor.get());
            SmartDashboard.getNumber("rightMotor speed", rightMotor.get());
        }

        // Update the current status of the forward and reverse limit switches.

        this.DS_ElevatorSetpoint.setDouble(elevatorController.getSetpoint());

        SmartDashboard.putData(this);
        SmartDashboard.putBoolean("elevator limit", limitSwitch.get()); */

        // The periodic method is called to regularly update the robot's status.
    }
}
