package frc.robot.subsystems.elevator;

/**
 * The ElevatorIOInputs class represents the input data for the Elevator subsystem. It contains fields for sensor
 * readings and other state information, which are updated by the hardware or simulation implementation of the
 * ElevatorIO interface.
 */
public class ElevatorIOInputs {
    public double position; // The current position of the elevator in meters or encoder units
    public boolean limitSwitch; // The state of the limit switch (true if triggered)
    public double velocity; // The current velocity of the elevator in meters per second or encoder units per second
    public double appliedVolts = 0.0; // The voltage currently applied to the motor
    public boolean limitSwitchPressed = false; // Indicates whether the limit switch is currently pressed.
    // This is useful for detecting when the elevator reaches a physical limit.
    public double kfValue = 0.0; // The feedforward gain for the elevator.
    // This value is used to apply a constant voltage to counteract gravity or assist the PID controller.
    public double kpValue = 0.0;
    // The proportional gain for the elevator.
    // This value is part of the PID controller and determines how strongly the system reacts to position errors.
    /** Constructor for the ElevatorIOInputs class. Initializes the fields to their default values. */
    public ElevatorIOInputs() {
        position = 0.0; // Default position is 0.0
        limitSwitch = false; // Default limit switch state is false (not triggered)
        velocity = 0.0; // Default velocity is 0.0
    }
}

/* GitHub Copilot
Here is the ElevatorIOInputs.java file with detailed comments added to explain each part of the code:

Explanation of the Code:

1. Purpose:

    The ElevatorIOInputs class is a data container for the elevator subsystem's state. It is used to store and update sensor readings such as position, velocity, and limit switch state.

2. Fields:

    position:
        Represents the current position of the elevator. This could be in meters (if converted) or raw encoder units.
    limitSwitch:
        Indicates whether the limit switch is triggered. This is useful for detecting when the elevator reaches its lower limit.
    velocity:
        Represents the current velocity of the elevator. This could be in meters per second or raw encoder units per second.

3. Constructor:

    Initializes all fields to their default values:
        position is set to 0.0, indicating the elevator starts at the bottom or a neutral position.
        limitSwitch is set to false, indicating the limit switch is not triggered by default.
        velocity is set to 0.0, indicating the elevator is stationary by default.

4. Usage:

    This class is typically updated by the updateInputs method in the ElevatorIO interface implementation (e.g., ElevatorIOReal or ElevatorIOSim).
    It provides a centralized structure for accessing the elevator's state, making it easier to manage and debug.


Why This is Useful:
    Centralizing the elevator's state in a single class makes the code cleaner and easier to maintain.
    It allows for consistent access to sensor data across different implementations (real hardware or simulation).
    The default values ensure the system starts in a known state, reducing the risk of uninitialized variables.
    Let me know if you need further clarification!

*/
