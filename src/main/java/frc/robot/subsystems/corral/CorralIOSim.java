package frc.robot.subsystems.corral;

public class CorralIOSim implements CorralIO {
    private double speed = 0.0;

    @Override
    public void updateInputs(CorralIOInputs inputs) {
        inputs.motorSpeed = speed;
        inputs.motorRunning = speed != 0.0;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void stop() {
        this.speed = 0.0;
    }
}
