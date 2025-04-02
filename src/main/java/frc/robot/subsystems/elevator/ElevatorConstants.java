package frc.robot.subsystems.elevator;

/**
 * The ElevatorConstants class contains constants used by the Elevator subsystem. These constants include hardware IDs,
 * physical constraints, PID values, and feedforward coefficients. They ensure consistency and make the code easier to
 * maintain and adjust.
 */
public class ElevatorConstants {
    // Hardware IDs
    public static final int ELEVATOR_L_CAN_ID = 11; // CAN ID for the left elevator motor (replace with actual ID)
    public static final int ELEVATOR_R_CAN_ID = 12; // CAN ID for the right elevator motor (replace with actual ID)
    public static final int LOWER_LIMIT_SWITCH_PORT =
            4; // DIO port for the lower limit switch (replace with actual port)

    // Current limits
    public static final int ELEVATOR_CURRENT_LIMITS = 40; // Maximum current limit for the elevator motors (in amps)

    // Physical constraints
    public static final double MAX_POS = 218; // Maximum position of the elevator in encoder units
    public static final double MIN_POS = 0; // Minimum position of the elevator in encoder units

    // Predefined positions for the elevator
    public static final double LOAD_POS = 0; // Position for loading (set this value based on your mechanism)
    public static final double L1_POS = 0; // Position for Level 1 (verify this value)
    public static final double L2_POS = 0; // Position for Level 2 (verify this value)
    public static final double L3_POS = 133.998; // Position for Level 3 (verify this value)
    public static final double L4_POS = 218; // Position for Level 4 (verify this value)

    // PID controller constants
    public static final double P = 0.08; // Proportional gain for the PID controller
    public static final double I = 0; // Integral gain for the PID controller
    public static final double D = 0; // Derivative gain for the PID controller
    public static final double PID_TOLERANCE =
            1; // Tolerance for considering the elevator "at setpoint" (in encoder units)

    // Feedforward coefficients
    public static final double KF_BELOW_0_5 = 0.01; // Feedforward coefficient for positions below 0.5 units
    public static final double KF_ABOVE_0_5 = 0.025; // Feedforward coefficient for positions above 0.5 units

    // Throttle divisor
    public static final double THROTTLE_DIVISOR = 65.71; // Divisor for calculating drive throttle
}

/* GitHub Copilot
Here is the ElevatorConstants.java file with detailed comments added to explain each constant:

Explanation of the Constants:

1. Hardware IDs:

ELEVATOR_L_CAN_ID and ELEVATOR_R_CAN_ID:
    These are the CAN IDs for the left and right elevator motors. Replace these values with the actual IDs assigned to the motors in your robot's configuration.

LOWER_LIMIT_SWITCH_PORT:
    This is the DIO port for the lower limit switch, which detects when the elevator reaches its lowest position.

2. Current Limits:

    ELEVATOR_CURRENT_LIMITS:
    Specifies the maximum allowable current for the elevator motors to prevent overheating or damage.

3. Physical Constraints:

    MAX_POS and MIN_POS:
    Define the maximum and minimum positions of the elevator in encoder units. These values ensure the elevator does not exceed its physical limits.

4. Predefined Positions:

    LOAD_POS, L1_POS, L2_POS, L3_POS, and L4_POS:
    These are predefined positions for the elevator, corresponding to specific levels or tasks (e.g., loading or scoring). Verify these values based on your robot's design.

5. PID Controller Constants:

    P, I, and D:
         These are the proportional, integral, and derivative gains for the PID controller, which is used to control the elevator's position.
    PID_TOLERANCE:
        Specifies how close the elevator must be to the target position to be considered "at setpoint."

6. Feedforward Coefficients:

    KF_BELOW_0_5 and KF_ABOVE_0_5:
        These coefficients are used to calculate a feedforward term to assist the PID controller in moving the elevator. The values differ based on the elevator's position.

7. Throttle Divisor:

THROTTLE_DIVISOR:
    Used to scale the drive throttle for smooth operation. Adjust this value based on your robot's behavior.


Why This is Useful:
    Centralizing constants in a single class makes the code easier to maintain and adjust.
    These constants ensure consistency across the Elevator subsystem and reduce the risk of hardcoding values in multiple places.
    By using descriptive names and comments, the purpose of each constant is clear, making the code more readable and maintainable. */
