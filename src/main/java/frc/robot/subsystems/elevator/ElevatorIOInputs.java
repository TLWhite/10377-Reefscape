package frc.robot.subsystems.elevator;

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
