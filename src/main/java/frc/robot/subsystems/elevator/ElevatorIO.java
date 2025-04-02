package frc.robot.subsystems.elevator;

/**
 * The ElevatorIO interface defines the hardware abstraction layer for the Elevator subsystem. It provides methods for
 * controlling the elevator and retrieving its state, allowing for different implementations (e.g., real hardware or
 * simulation).
 */
public interface ElevatorIO {
    /**
     * Updates the input data structure with the latest sensor readings.
     *
     * @param inputs The input data structure to update.
     */
    void updateInputs(ElevatorIOInputs inputs);

    /**
     * Sets the target position of the elevator.
     *
     * @param position The target position in meters.
     */
    void setPosition(double position);

    /**
     * Gets the current position of the elevator.
     *
     * @return The position of the elevator in meters.
     */
    double getPosition();

    /**
     * Sets the voltage to be applied to the elevator motor.
     *
     * @param voltage The voltage to apply to the motor.
     */
    void setVoltage(double voltage);

    /**
     * Gets the current velocity of the elevator.
     *
     * @return The velocity of the elevator in meters per second.
     */
    double getVelocity();

    /**
     * The ElevatorIOInputs class represents the input data for the Elevator subsystem. It contains fields for sensor
     * readings and other state information.
     */
    public class ElevatorIOInputs {
        public double position; // The current position of the elevator in meters
        public boolean limitSwitch; // The state of the limit switch (true if triggered)
        public double velocity; // The current velocity of the elevator in meters per second

        /** Constructor for the ElevatorIOInputs class. Initializes the fields to their default values. */
        public ElevatorIOInputs() {
            position = 0.0; // Default position is 0.0 meters
            limitSwitch = false; // Default limit switch state is false (not triggered)
            velocity = 0.0; // Default velocity is 0.0 meters per second
        }
    }
}

/*
1.Interface Overview:

The ElevatorIO interface defines the contract for interacting with the elevator hardware or simulation.
It abstracts the hardware details, allowing for flexibility in implementation (e.g., real hardware or simulation).

2.Methods:

    updateInputs(ElevatorIOInputs inputs):
        Updates the ElevatorIOInputs object with the latest sensor data, such as position, velocity, and limit switch state.
    setPosition(double position):
        Sets the target position of the elevator in meters.
    getPosition():
        Retrieves the current position of the elevator in meters.
    setVoltage(double voltage):
        Sends a voltage command to the elevator motor.
    getVelocity():
        Retrieves the current velocity of the elevator in meters per second.

3. ElevatorIOInputs Class:

    This nested class is a data container for the elevator's state.
    It includes:
        position: The current position of the elevator in meters.
        limitSwitch: The state of the limit switch (true if triggered).
        velocity: The current velocity of the elevator in meters per second.
    The constructor initializes these fields to default values.

4. Why This is Useful:

    The ElevatorIO interface allows for a clean separation between the hardware control logic and the rest of the robot code.
    By using this interface, the elevator subsystem can easily switch between real hardware and simulation implementations without changing the rest of the code. */
