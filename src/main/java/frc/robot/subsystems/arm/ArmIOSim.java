package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getVex775Pro(2), // Motor type with 2 motors
            ArmConstants.GEAR_RATIO, // Gear ratio
            SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH, ArmConstants.ARM_MASS), // Moment of Inertia
            ArmConstants.ARM_MASS, // Arm mass
            ArmConstants.MIN_POS, // Minimum position (radians)
            ArmConstants.MAX_POS, // Maximum position (radians)
            false, // Simulate gravity
            ArmConstants.ARM_LENGTH // Arm length (meters)
            );
    // private final DIOSim limitSwitchSim = new DIOSim(ArmConstants.LIMIT_SWITCH_PORT);
    private double appliedVolts = 0.0;

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public double getPosition() {
        return sim.getAngleRads() / (2 * Math.PI); // Convert radians to rotations
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec() / (2 * Math.PI); // Convert rad/s to rotations/s
    }

    @Override
    public boolean getLimitSwitch() {
        return sim.getAngleRads() <= ArmConstants.MIN_POS * 2 * Math.PI;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        sim.update(0.02); // 20ms loop
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.appliedVolts = appliedVolts;
        inputs.limitSwitch = getLimitSwitch();
    }
}
