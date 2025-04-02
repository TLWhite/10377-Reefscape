package frc.robot.subsystems.corral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Corral extends SubsystemBase {
    private final CorralIO io;
    private final CorralIOInputsAutoLogged inputs = new CorralIOInputsAutoLogged();

    public Corral(CorralIO io) {
        this.io = io;
        Logger.processInputs("Corral", inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Corral", inputs);
    }

    public void run(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.setVoltage(0.0);
    }
}
