package frc.robot.subsystems.arm;

public class ArmConstants {
    public static final int ARM_MOTOR_ID = 13; // TODO: Replace with your actual CAN ID
    public static final int ENCODER_PORT = 2; // Updated with the actual DIO port
    public static final int LIMIT_SWITCH_PORT = 4; // TODO: Verify this value
    public static final boolean USE_LIMIT_SWITCH = false; // TODO: Change this to True if limit switch is installed.
    public static final double ENCODER_OFFSET = 0.0; // adjust as needed

    public static final double L0_POS = 0.2; // TODO: Verify
    public static final double L1_POS = 0.25; // TODO: Verify
    public static final double L2_POS = 0.179; // TODO: Verify
    public static final double L3_POS = 0.1365; // TODO: Verify
    public static final double L4_POS =
            0.0887; // TODO: Verify    public static final double MIN_POS = -0.25; // Min arm position (TODO: Tune)

    public static final double P = 3.4; // PID Proportional gain
    public static final double I = 0; // PID Integral gain
    public static final double D = 0; // PID Derivative gain
    public static final double PID_TOLERANCE = 0.0075; // PID tolerance
    public static final double KF_COEFFICIENT = 0.10; // Feedforward coefficient (TODO: Tune)

    public static final double GEAR_RATIO = 1.0 / 3.0; // Gear ratio
    public static final double ARM_LENGTH = 0.5; // Arm length (meters)
    public static final double ARM_MASS = 1.0; // Arm mass (kg)
    public static final double GRAVITY = 9.8; // Gravity constant (m/s^2)

    // Define safe position range constants
    public static final double MIN_POS = 0.0; // Minimum allowable arm position
    public static final double MAX_POS = 1.0; // M

    public static final double MIN_SAFE_FOLD_POS = -0.2; // Replace with the correct value
    public static final double MAX_SAFE_FOLD_POS = -0.055; // Replace with the correct value

    /*     private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");
       private final GenericEntry kPEntry = tab.add("Arm kP", 0.0).getEntry();
       private final GenericEntry kIEntry = tab.add("Arm kI", 0.0).getEntry();
       private final GenericEntry kDEntry = tab.add("Arm kD", 0.0).getEntry();
    */
    // Update the PID constants based on Shuffleboard inputs
}
