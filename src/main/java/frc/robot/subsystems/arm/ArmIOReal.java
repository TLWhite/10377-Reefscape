package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX; // For controlling the TalonFX motor controller
import edu.wpi.first.wpilibj.DigitalInput; // For reading the state of a digital input (e.g., limit switch)
import edu.wpi.first.wpilibj.DutyCycleEncoder; // For reading the position of a duty cycle encoder

/**
 * The ArmIOReal class implements the ArmIO interface for real hardware. It provides methods to interact with the
 * physical components of the arm, such as the motor, encoder, and limit switch.
 */
public class ArmIOReal implements ArmIO {
    // Motor controller for the arm
    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    // Encoder for measuring the arm's position
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);

    // Limit switch for detecting the arm's physical limits
    private final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

    /**
     * Constructor for the ArmIOReal class. Initializes the hardware components. No specific motor configuration is
     * needed in this case.
     */
    public ArmIOReal() {
        // No specific motor configuration needed for TalonFX in this case
    }

    /**
     * Sets the voltage to be applied to the arm motor.
     *
     * @param volts The voltage to apply to the motor.
     */
    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    /**
     * Gets the current position of the arm from the encoder. The encoder value is adjusted to handle wraparound
     * behavior.
     *
     * @return The position of the arm (e.g., in rotations or encoder units).
     */
    @Override
    public double getPosition() {
        double rawValue = -encoder.get(); // Get the raw encoder value (negative for direction adjustment)

        // Handle wraparound: if the value is less than -0.5, adjust it by adding 1.0
        if (rawValue < -0.5) {
            rawValue += 1.0;
        }

        return rawValue;
    }

    /**
     * Gets the current velocity of the arm from the motor controller.
     *
     * @return The velocity of the arm in rotations per second.
     */
    @Override
    public double getVelocity() {
        return armMotor.getVelocity().getValueAsDouble(); // TalonFX provides velocity in rotations per second
    }

    /**
     * Checks the state of the limit switch.
     *
     * @return True if the limit switch is triggered, false otherwise.
     */
    @Override
    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    /**
     * Updates the input data structure with the latest sensor readings.
     *
     * @param inputs The input data structure to update.
     */
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.position = getPosition(); // Update the position field with the current encoder value
        inputs.velocity = getVelocity(); // Update the velocity field with the current motor velocity
        inputs.appliedVolts = armMotor.getMotorVoltage().getValueAsDouble(); // Update the applied voltage
        inputs.limitSwitch = getLimitSwitch(); // Update the limit switch state
    }
}

/*


1.Class Overview:
The ArmIOReal class implements the ArmIO interface and provides real hardware implementations for controlling the arm.

2.Fields:
    armMotor: Represents the TalonFX motor controller used to drive the arm.
    encoder: Reads the position of the arm using a duty cycle encoder.
    limitSwitch: Detects whether the arm has reached a physical limit.

3. Methods:
    setVoltage(double volts): Sends a voltage command to the motor.
    getPosition(): Retrieves the arm's position from the encoder and adjusts for wraparound behavior.
    getVelocity(): Retrieves the arm's velocity from the motor controller.
    getLimitSwitch(): Checks whether the limit switch is triggered.
    updateInputs(ArmIOInputs inputs): Updates the ArmIOInputs object with the latest sensor data, ensuring the subsystem has up-to-date information.

4. Wraparound Handling in getPosition():
    The encoder value is adjusted to handle wraparound behavior. If the raw value is less than -0.5, it is incremented by 1.0 to ensure continuity.*/
