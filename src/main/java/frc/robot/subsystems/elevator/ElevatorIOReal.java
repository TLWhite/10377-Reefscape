package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder; // For reading encoder values
import com.revrobotics.spark.SparkBase.PersistMode; // For configuring persistence of motor settings
import com.revrobotics.spark.SparkBase.ResetMode; // For configuring reset behavior of motor settings
import com.revrobotics.spark.SparkMax; // For controlling SparkMax motor controllers
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // For setting motor idle mode
import com.revrobotics.spark.config.SparkMaxConfig; // For configuring SparkMax motor controllers
import edu.wpi.first.wpilibj.DigitalInput; // For reading the state of a digital input (e.g., limit switch)

/**
 * The ElevatorIOReal class implements the ElevatorIO interface for real hardware. It provides methods to interact with
 * the physical components of the elevator subsystem, such as motors, encoders, and limit switches.
 */
public class ElevatorIOReal implements ElevatorIO {
    // Motor controllers for the elevator
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.ELEVATOR_L_CAN_ID, SparkMax.MotorType.kBrushless);
    private final SparkMax rightMotor =
            new SparkMax(ElevatorConstants.ELEVATOR_R_CAN_ID, SparkMax.MotorType.kBrushless);

    // Encoder for measuring the elevator's position and velocity
    private final RelativeEncoder encoder = rightMotor.getEncoder();

    // Limit switch for detecting the elevator's lower limit
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_PORT);

    /**
     * Constructor for the ElevatorIOReal class. Configures the motor controllers and initializes hardware components.
     */
    public ElevatorIOReal() {
        // Create a configuration object for the SparkMax motor controllers
        SparkMaxConfig config = new SparkMaxConfig();

        // Configure the motor settings
        config.idleMode(IdleMode.kBrake) // Set the motor to brake mode when idle
                .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMITS) // Set current limits
                .inverted(true); // Invert the direction of the left motor

        // Configure soft limits for the elevator's movement
        config.softLimit
                .forwardSoftLimit(ElevatorConstants.MAX_POS) // Set the forward soft limit
                .forwardSoftLimitEnabled(false) // Disable the forward soft limit
                .reverseSoftLimit(ElevatorConstants.MIN_POS) // Set the reverse soft limit
                .reverseSoftLimitEnabled(false); // Disable the reverse soft limit

        // Apply the configuration to the left motor
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Update the configuration for the right motor (not inverted)
        config.inverted(false);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the voltage to be applied to the elevator motors.
     *
     * @param volts The voltage to apply to the motors.
     */
    @Override
    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts); // Apply voltage to the left motor
        rightMotor.setVoltage(volts); // Apply voltage to the right motor
    }

    /**
     * Gets the current position of the elevator from the encoder.
     *
     * @return The position of the elevator in encoder units.
     */
    @Override
    public double getPosition() {
        return encoder.getPosition(); // Retrieve the current position from the encoder
    }

    /**
     * Sets the position of the elevator encoder.
     *
     * @param position The position to set in encoder units.
     */
    @Override
    public void setPosition(double position) {
        encoder.setPosition(position); // Set the encoder's position
    }

    /**
     * Updates the input data structure with the latest sensor readings.
     *
     * @param inputs The input data structure to update.
     */
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = getPosition(); // Update the position field with the current encoder value
        inputs.limitSwitch = !limitSwitch.get(); // Update the limit switch state (inverted logic)
    }

    /**
     * Gets the current velocity of the elevator from the encoder.
     *
     * @return The velocity of the elevator in encoder units per second.
     */
    @Override
    public double getVelocity() {
        return encoder.getVelocity(); // Retrieve the current velocity from the encoder
    }
}

/*
Explanation of the Code:
1. Purpose:
    The ElevatorIOReal class is a real hardware implementation of the ElevatorIO interface. It interacts with the physical components of the elevator subsystem, such as motors, encoders, and limit switches.

2. Hardware Components:
    SparkMax Motor Controllers:
        Two SparkMax motor controllers (leftMotor and rightMotor) are used to drive the elevator.
    RelativeEncoder:
        The encoder is used to measure the elevator's position and velocity.
    DigitalInput:
        The limit switch detects when the elevator reaches its lower limit.

3. Configuration:
        The SparkMaxConfig object is used to configure the motor controllers:
             Idle Mode: Sets the motors to brake mode when idle to prevent unwanted movement.
             Current Limits: Limits the current to protect the motors and electronics.
             Soft Limits: Defines forward and reverse limits for the elevator's movement, though they are disabled in this implementation.

4. Key Methods:

    setVoltage(double volts):
         Sends a voltage command to both motors to control the elevator's movement.
    getPosition():
         Retrieves the current position of the elevator from the encoder.
    setPosition(double position):
        Sets the encoder's position, which can be useful for resetting or calibrating the elevator.

   */
