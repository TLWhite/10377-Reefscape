package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor; // For simulating DC motors
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim; // For simulating a single-jointed arm

/**
 * The ArmIOSim class implements the ArmIO interface for simulation purposes. It uses WPILib's SingleJointedArmSim to
 * simulate the behavior of the arm, including its position, velocity, and limit switch state.
 */
public class ArmIOSim implements ArmIO {
    // Simulates the arm mechanism using a single-jointed arm model
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getVex775Pro(2), // Simulates two Vex 775 Pro motors
            ArmConstants.GEAR_RATIO, // Gear ratio of the arm mechanism
            SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH, ArmConstants.ARM_MASS), // Moment of Inertia
            ArmConstants.ARM_MASS, // Mass of the arm in kilograms
            ArmConstants.MIN_POS, // Minimum position of the arm in radians
            ArmConstants.MAX_POS, // Maximum position of the arm in radians
            false, // Gravity simulation is disabled
            ArmConstants.ARM_LENGTH // Length of the arm in meters
            );

    // Simulated voltage applied to the motor
    private double appliedVolts = 0.0;

    /**
     * Sets the voltage to be applied to the simulated arm motor.
     *
     * @param volts The voltage to apply to the motor.
     */
    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts; // Store the applied voltage
        sim.setInputVoltage(volts); // Apply the voltage to the simulation
    }

    /**
     * Gets the current position of the simulated arm.
     *
     * @return The position of the arm in rotations (converted from radians).
     */
    @Override
    public double getPosition() {
        return sim.getAngleRads() / (2 * Math.PI); // Convert radians to rotations
    }

    /**
     * Gets the current velocity of the simulated arm.
     *
     * @return The velocity of the arm in rotations per second (converted from radians per second).
     */
    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec() / (2 * Math.PI); // Convert rad/s to rotations/s
    }

    /**
     * Checks the state of the simulated limit switch.
     *
     * @return True if the arm is at or below the minimum position, false otherwise.
     */
    @Override
    public boolean getLimitSwitch() {
        return sim.getAngleRads() <= ArmConstants.MIN_POS * 2 * Math.PI; // Check if the arm is at the minimum position
    }

    /**
     * Updates the input data structure with the latest simulation values.
     *
     * @param inputs The input data structure to update.
     */
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        sim.update(0.02); // Simulate a 20ms loop (typical for robot control systems)
        inputs.position = getPosition(); // Update the position field
        inputs.velocity = getVelocity(); // Update the velocity field
        inputs.appliedVolts = appliedVolts; // Update the applied voltage field
        inputs.limitSwitch = getLimitSwitch(); // Update the limit switch state
    }
}

/*

1.Purpose:

    The ArmIOSim class is a simulation implementation of the ArmIO interface. It is used to simulate the behavior of the arm subsystem in a virtual environment, allowing for testing and debugging without requiring physical hardware.

2. Simulation Model:
The SingleJointedArmSim class from WPILib is used to simulate the arm's motion. It models the arm as a single-jointed mechanism with parameters such as motor type, gear ratio, moment of inertia, arm mass, and arm length.

3.Key Methods:
    setVoltage(double volts):
    Simulates applying a voltage to the motor. The voltage is stored in the appliedVolts field and passed to the simulation model.
    getPosition():
    Retrieves the arm's current position in rotations by converting the simulation's angle (in radians) to rotations.
    getVelocity():
    Retrieves the arm's current velocity in rotations per second by converting the simulation's velocity (in radians per second) to rotations per second.
    getLimitSwitch():
    Simulates the behavior of a limit switch. It returns true if the arm's position is at or below the minimum position.
    updateInputs(ArmIOInputs inputs):
    Updates the ArmIOInputs object with the latest simulation data, including position, velocity, applied voltage, and limit switch state. The simulation is updated every 20ms to mimic a real robot control loop.

4. Simulation Parameters:
    DCMotor.getVex775Pro(2):
    Simulates two Vex 775 Pro motors driving the arm.
    ArmConstants.GEAR_RATIO:
    Represents the gear ratio of the arm mechanism, which affects torque and speed.
    SingleJointedArmSim.estimateMOI():
    Estimates the moment of inertia of the arm based on its length and mass.
    ArmConstants.MIN_POS and ArmConstants.MAX_POS:
    Define the physical limits of the arm's motion in radians.
    ArmConstants.ARM_LENGTH:
    Represents the length of the arm in meters, which affects the simulation's physics.

5. Why This is Useful:
    The ArmIOSim class allows developers to test and debug the arm subsystem in a simulated environment. This is particularly useful when physical hardware is unavailable or when testing edge cases that could damage the hardware.
    Let me know if you need further clarification! */
