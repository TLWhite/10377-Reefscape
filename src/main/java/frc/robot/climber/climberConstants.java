package frc.robot.climber;

/**
 * The ClimberConstants class contains constants for the climber subsystem, which may include motors for lifting or
 * extending the robot during climbing, as well as limit switches for detecting positions.
 */
public class climberConstants {

    // CAN ID for the motor controlling the climber
    public static final int ClimberMotor_ID = 18;

    // CAN ID for the climber encoder (used to measure the climber's position)
    public static final int ClimberCANcoder_ID = 20;

    // Pin numbers for high and low limit switches on the climber
    public static final int HILimitPin = 5;
    public static final int LOLimitPin = 4;

    // Pin number for controlling the intake tray (for raising or lowering
    // it)
    public static final int IntakeTray_Pin = 1;
}
