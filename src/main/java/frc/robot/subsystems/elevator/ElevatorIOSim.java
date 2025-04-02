package frc.robot.subsystems.elevator;

// import edu.wpi.first.wpilibj.simulation.DIOSim; // Uncomment if simulating a digital input for a limit switch
import edu.wpi.first.math.system.plant.DCMotor; // For simulating DC motors
import edu.wpi.first.wpilibj.simulation.ElevatorSim; // For simulating an elevator mechanism

/**
 * The ElevatorIOSim class implements the ElevatorIO interface for simulation purposes. It uses WPILib's ElevatorSim to
 * simulate the behavior of the elevator subsystem, including its position, velocity, and response to voltage inputs.
 */
public class ElevatorIOSim implements ElevatorIO {
    // Simulates the elevator mechanism using WPILib's ElevatorSim
    private final ElevatorSim sim = new ElevatorSim(
            DCMotor.getNEO(2), // Simulates two NEO motors (approximation for SparkMax motor controllers)
            10.0, // Gear ratio of the elevator mechanism
            5.0, // Carriage mass in kilograms (adjusted based on the elevator's design)
            0.02, // Drum radius in meters (adjusted based on the mechanism)
            ElevatorConstants.MIN_POS, // Minimum position of the elevator in meters
            ElevatorConstants.MAX_POS, // Maximum position of the elevator in meters
            true, // Simulate gravity acting on the elevator
            0.0 // Minimum velocity (adjusted as needed)
            );

    /**
     * Updates the input data structure with the latest simulation values.
     *
     * @param inputs The input data structure to update.
     */
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = sim.getPositionMeters(); // Update the position field with the current simulated position
        inputs.velocity =
                sim.getVelocityMetersPerSecond(); // Update the velocity field with the current simulated velocity
    }

    /**
     * Sets the target position of the simulated elevator.
     *
     * @param position The target position in meters.
     */
    @Override
    public void setPosition(double position) {
        sim.setInput(position); // Set the target position for the simulation
    }

    /**
     * Gets the current position of the simulated elevator.
     *
     * @return The position of the elevator in meters.
     */
    @Override
    public double getPosition() {
        return sim.getPositionMeters(); // Retrieve the current simulated position
    }

    /**
     * Sets the voltage to be applied to the simulated elevator motor.
     *
     * @param voltage The voltage to apply to the motor.
     */
    @Override
    public void setVoltage(double voltage) {
        sim.setInput(voltage); // Apply the voltage to the simulation
    }

    /**
     * Gets the current velocity of the simulated elevator.
     *
     * @return The velocity of the elevator in meters per second.
     */
    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond(); // Retrieve the current simulated velocity
    }
}

/*

 1. Purpose:

	The ElevatorIOSim class is a simulation implementation of the ElevatorIO interface. It is used to simulate the behavior of the elevator subsystem in a virtual environment, allowing for testing and debugging without requiring physical hardware.
2. Simulation Model:

	The ElevatorSim class from WPILib is used to simulate the elevator's motion. It models the elevator as a mechanism driven by motors, with parameters such as gear ratio, carriage mass, and drum radius.

3. Key Methods:

	updateInputs(ElevatorIOInputs inputs):
	Updates the ElevatorIOInputs object with the latest simulation data, including position and velocity.
	setPosition(double position):
	Sets the target position of the elevator in the simulation.
	getPosition():
	Retrieves the current position of the elevator in meters.
	setVoltage(double voltage):
	Simulates applying a voltage to the elevator motor.
	getVelocity():
	Retrieves the current velocity of the elevator in meters per second.

4. Simulation Parameters:

	DCMotor.getNEO(2):
	Simulates two NEO motors driving the elevator.
	10.0 (Gear Ratio):
	Represents the gear ratio of the elevator mechanism, which affects torque and speed.
	5.0 (Carriage Mass):
	Represents the mass of the elevator carriage in kilograms.
	0.02 (Drum Radius):
	Represents the radius of the drum or pulley in meters.
	ElevatorConstants.MIN_POS and ElevatorConstants.MAX_POS:
	Define the physical limits of the elevator's motion in meters.
	true (Simulate Gravity):
	Enables gravity simulation, which affects the elevator's motion.

5.Why This is Useful:

	The ElevatorIOSim class allows developers to test and debug the elevator subsystem in a simulated environment. This is particularly useful when physical hardware is unavailable or when testing edge cases that could damage the hardware.

    */
