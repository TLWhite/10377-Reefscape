package frc.robot.subsystems.arm;

/**
 * This class contains constants used by the Arm subsystem. These constants include PID values, setpoints, physical
 * constraints, and hardware configurations.
 */
public class ArmConstants {
    // PID controller constants for controlling the arm's position
    public static final double P = 8.0; // Proportional gain for the PID controller
    public static final double I = 0.0; // Integral gain for the PID controller
    public static final double D = 0.2; // Derivative gain for the PID controller
    public static final double PID_TOLERANCE = 0.01; // Tolerance for considering the arm "at setpoint"

    // Setpoints for predefined arm positions
    public static final double L0_POS = 0.0; // Position for Level 0
    public static final double L1_POS = 0.5; // Position for Level 1
    public static final double L2_POS = 0.5; // Position for Level 2
    public static final double L3_POS = 0.6; // Position for Level 3
    public static final double L4_POS = 0.65; // Position for Level 4
    public static final double LOAD_POS = 0.25; // Approximate position for loading

    // Safe range for folding the arm
    // Ensure the encoder does not produce negative values
    public static final double MIN_SAFE_FOLD_POS = 0.0; // Minimum safe position for folding
    public static final double MAX_SAFE_FOLD_POS = 0.99; // Maximum safe position for folding

    // Physical constraints of the arm
    public static final double MIN_POS = 0.00; // Minimum physical position of the arm
    public static final double MAX_POS = 0.75; // Maximum physical position of the arm (estimated)

    // Feedforward coefficient for compensating gravity
    public static final double KF_COEFFICIENT = 0.10; // Adjust based on testing

    // Hardware IDs for the arm components
    public static final int ARM_MOTOR_ID = 31; // CAN ID for the arm motor
    public static final int LIMIT_SWITCH_PORT = 1; // Digital input port for the limit switch

    // Encoder configuration
    public static final boolean USE_LIMIT_SWITCH = false; // Whether to use the limit switch
    public static final double ENCODER_OFFSET = 0.24; // Offset for the encoder position
    public static final int ENCODER_PORT = 0; // Port for the encoder

    // Simulation and physical properties of the arm
    public static final double GEAR_RATIO = 100.0; // Gear ratio of the arm mechanism
    public static final double ARM_MASS = 3.0; // Mass of the arm in kilograms (adjust as needed)
    public static final double ARM_LENGTH = 0.5; // Length of the arm in meters
}
