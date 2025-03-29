package frc.robot.subsystems.elevator;

// import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim = new ElevatorSim(
            DCMotor.getNEO(2), // 2 NEO motors (approximation for SparkMax)
            10.0, // Gear ratio (adjusted based on your mechanism)
            5.0, // Carriage mass in kg (adjusted)
            0.02, // Drum radius in meters (adjusted)
            ElevatorConstants.MIN_POS,
            ElevatorConstants.MAX_POS,
            true, // Simulate gravity
            0.0 // Minimum velocity (adjusted as needed)
            );

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Implementation for updating inputs
        inputs.position = sim.getPositionMeters();
        inputs.velocity = sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setPosition(double position) {
        // Implementation for setting position
        sim.setInput(position);
    }

    @Override
    public double getPosition() {
        // Implementation for getting position
        return sim.getPositionMeters();
    }

    @Override
    public void setVoltage(double voltage) {
        // Implementation for setting voltage
        sim.setInput(voltage);
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }
}
