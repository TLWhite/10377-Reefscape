package frc.robot.subsystems.arm;

/**
 * The ArmIO interface defines the hardware abstraction layer for the Arm subsystem. It provides methods for controlling
 * the arm and retrieving its state, allowing for different implementations (e.g., real hardware or simulation).
 */
public interface ArmIO {
    /**
     * Sets the voltage to be applied to the arm motor.
     *
     * @param volts The voltage to apply to the motor.
     */
    void setVoltage(double volts);

    /**
     * Gets the current position of the arm.
     *
     * @return The position of the arm (e.g., in encoder units or radians).
     */
    double getPosition();

    /**
     * Gets the current velocity of the arm.
     *
     * @return The velocity of the arm (e.g., in encoder units per second or radians per second).
     */
    double getVelocity();

    /**
     * Checks the state of the limit switch.
     *
     * @return True if the limit switch is triggered, false otherwise.
     */
    boolean getLimitSwitch();

    /**
     * Updates the input data structure with the latest sensor readings.
     *
     * @param inputs The input data structure to update.
     */
    void updateInputs(ArmIOInputs inputs);

    /**
     * The ArmIOInputs class represents the input data for the Arm subsystem. It contains fields for sensor readings and
     * other state information.
     */
    public class ArmIOInputs {
        public double position = 0.0; // The current position of the arm
        public double velocity = 0.0; // The current velocity of the arm
        public double appliedVolts = 0.0; // The voltage currently applied to the motor
        public boolean limitSwitch = false; // The state of the limit switch (true if triggered)
        public double offset = 0.0; // The offset for the encoder position
    }
}

/* Interface Overview:

The ArmIO interface defines the contract for interacting with the arm hardware or simulation. It abstracts the hardware details, allowing for flexibility in implementation.
Methods:

setVoltage(double volts): Sends a voltage command to the arm motor.
getPosition(): Retrieves the current position of the arm, typically from an encoder.
getVelocity(): Retrieves the current velocity of the arm, typically from an encoder or derivative calculation.
getLimitSwitch(): Checks whether the limit switch is triggered, which can be used to detect the arm's physical limits.
updateInputs(ArmIOInputs inputs): Updates the ArmIOInputs object with the latest sensor data, ensuring the subsystem has up-to-date information.
ArmIOInputs Class:

This nested class is a data container for the arm's state, including:
position: The current position of the arm.
velocity: The current velocity of the arm.
appliedVolts: The voltage currently applied to the motor.
limitSwitch: The state of the limit switch.
offset: An offset value for adjusting the encoder's zero position. */
