package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOReal implements ArmIO {

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Populate the inputs object with data from the hardware
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.limitSwitch = getLimitSwitch();
    }

    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
    private final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

    public ArmIOReal() {
        // REV Through Bore gives 0.0 to 1.0; apply offset if needed
        encoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0); // Optional
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public double getPosition() {
        // REV encoder returns 0.0 to 1.0, so multiply by 2π for radians
        double position = encoder.get();
        return (position * 2.0 * Math.PI) + ArmConstants.ENCODER_OFFSET;
    }

    @Override
    public double getVelocity() {
        return armMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public boolean getLimitSwitch() {
        if (ArmConstants.USE_LIMIT_SWITCH) {
            return !limitSwitch.get(); // Assuming normally closed switch
        } else {
            // Simulate the switch being pressed near the bottom (e.g., within ±0.05 rad)
            return Math.abs(getPosition()) < 0.05;
        }
    }

    @Override
    public void zeroEncoder() {
        // This encoder does not support zeroing in hardware — do it with offset
        // Just log the current position to use in offset tuning
        System.out.println("Current encoder value: " + encoder.get());
    }
}
