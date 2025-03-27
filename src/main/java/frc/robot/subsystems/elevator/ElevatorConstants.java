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
    public static final int L1 = 5; // TODO set this value
    public static final int L2 = 10; //  TODO set this value
    public static final int L3 = 15; // TODO set this value
    public static final int L4 = 20; // TODO set this value

    public static final int downPos = 0;
    public static final int bottomPos = 0;

    public static final double minPos = 0; // TODO set this value
    public static final double maxPos = 10; // TODO set this value

    public static final int countsPerInch = 0;
    public static final int posTolerance = 0;

    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kI = 0.0;

    public static final int max_output = 0;

    public static final int elevatorLCanId = 11;
    public static final int elevatorRCanId = 12;

    public static final int maxVelocity = 0;
    public static final int maxAcceleration = 0;

    public static final int lowerLimitSwitchPort = 0; // TODO set this value
    public static int ELEVATOR_CURRENT_LIMITS = 0; // TODO set this value
}
