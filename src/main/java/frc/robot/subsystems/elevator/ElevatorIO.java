package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    void updateInputs(ElevatorIOInputs inputs);

    void setPosition(double position);

    double getPosition();

    void setVoltage(double voltage);

    double getVelocity(); // This method should be in the interface

    public class ElevatorIOInputs {
        public double position;
        public boolean limitSwitch;
        public double velocity;

        public ElevatorIOInputs() {
            position = 0.0;
            limitSwitch = false;
            velocity = 0.0;
        }
    }
}
