package frc.robot.subsystems.arm;

public interface ArmIO {
    void setVoltage(double volts);

    double getPosition();

    double getVelocity();

    boolean getLimitSwitch();

    void updateInputs(ArmIOInputs inputs);

    public class ArmIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public boolean limitSwitch = false;
    }
}
