package frc.robot.subsystems.arm;

public class ArmConstants {
    // PID
    public static final double P = 8.0;
    public static final double I = 0.0;
    public static final double D = 0.2;
    public static final double PID_TOLERANCE = 0.01;

    // Setpoints
    public static final double L0_POS = 0.2;
    public static final double L1_POS = 0.25;
    public static final double L2_POS = 0.179;
    public static final double L3_POS = 0.1365;
    public static final double L4_POS = 0.0887;
    public static final double LOAD_POS = -0.25; // Approximate load position

    // Safe range for folding
    public static final double MIN_SAFE_FOLD_POS = 0.05;
    public static final double MAX_SAFE_FOLD_POS = 0.22;

    // Physical constraints
    public static final double MIN_POS = 0.02;
    public static final double MAX_POS = 0.28;

    // Feedforward coefficient (adjust based on testing)
    public static final double KF_COEFFICIENT = 0.10;

    // Hardware IDs
    public static final int ARM_MOTOR_ID = 31;
    public static final int LIMIT_SWITCH_PORT = 1;

    // Encoder configuration
    public static final boolean USE_LIMIT_SWITCH = false;
    public static final double ENCODER_OFFSET = 0.0;
    public static final int ENCODER_PORT = 0;

    public static final double GEAR_RATIO = 100.0; // Replace with your actual gear ratio
    public static final double ARM_MASS = 3.0; // In kilograms (adjust as needed)
    public static final double ARM_LENGTH = 0.5;
}
