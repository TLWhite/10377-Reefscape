package frc.robot.subsystems.corral;

import com.ctre.phoenix6.hardware.TalonFX;

public class CorralIOReal implements CorralIO {
    private final TalonFX motor = new TalonFX(CorralConstants.MOTOR_ID);

    @Override
    public void updateInputs(CorralIOInputs inputs) {
        inputs.motorSpeed = motor.get();
        inputs.motorRunning = motor.get() != 0.0;
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
