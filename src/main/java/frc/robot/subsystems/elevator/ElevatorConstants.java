// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class ElevatorConstants {
    public static final int DOWN_POS = 0;
    public static final int BOTTOM_POS = 0;

    public static final double MIN_POS = 0; // TODO set this value
    public static final double MAX_POS = 10; // TODO set this value

    public static final int COUNTS_PER_INCH = 0;
    public static final int POS_TOLERANCE = 0;

    public static final double KP = 0.0;
    public static final double KD = 0.0;
    public static final double KI = 0.0;

    public static final int MAX_OUTPUT = 0;

    public static final int ELEVATOR_L_CAN_ID = 11;
    public static final int ELEVATOR_R_CAN_ID = 12;

    public static final int MAX_VELOCITY = 0;
    public static final int MAX_ACCELERATION = 0;

    public static final int LOWER_LIMIT_SWITCH_PORT = 0; // TODO set this value
    public static int ELEVATOR_CURRENT_LIMITS = 0; // TODO set this value
}
