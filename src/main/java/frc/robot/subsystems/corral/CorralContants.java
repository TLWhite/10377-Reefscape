package frc.robot.subsystems.corral;

/**
 * Formaly the EndEffectorsConstants class from 6424. defines constants for the endEffector subsystem, including CAN IDs
 * for the EndEffector motor and pin assignments for beam break sensors.
 */
public class CorralContants {
    // CAN ID for the motor controlling the endEffector mechanism
    public static final int IntakeCAN_ID = 17;

    // Pin numbers for beam break sensors used to detect objects being picked up or
    // released
    public static final int BeamBreakPinIntake = 2;
    public static final int BeamBreakPinDis = 1;
}
