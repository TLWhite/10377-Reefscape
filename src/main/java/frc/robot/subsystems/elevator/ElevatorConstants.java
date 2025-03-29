package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final int ELEVATOR_L_CAN_ID = 1; // TODO: Replace with your actual CAN ID
    public static final int ELEVATOR_R_CAN_ID = 2; // TODO: Replace with your actual CAN ID
    public static final int LOWER_LIMIT_SWITCH_PORT = 0; // TODO: Replace with your actual DIO port
    public static final int ELEVATOR_CURRENT_LIMITS = 40; // TODO: Verify or adjust this value
    public static final double MAX_POS = 218; // Max position in encoder units
    public static final double MIN_POS = 0; // Min position in encoder units
    public static final double LOAD_POS = 0; // TODO: Set this value
    public static final double L1_POS = 10.88; // TODO: Verify this value
    public static final double L2_POS = 66.43; // TODO: Verify this value
    public static final double L3_POS = 133.998; // TODO: Verify this value
    public static final double L4_POS = 218; // TODO: Verify this value
    public static final double P = 0.08; // PID Proportional gain
    public static final double I = 0; // PID Integral gain
    public static final double D = 0; // PID Derivative gain
    public static final double PID_TOLERANCE = 1; // PID tolerance in encoder units
    public static final double KF_BELOW_0_5 = 0.01; // Feedforward below 0.5 units
    public static final double KF_ABOVE_0_5 = 0.025; // Feedforward above 0.5 units
    public static final double THROTTLE_DIVISOR = 65.71; // For drive throttle calculation
}
