package frc.robot.subsystems.corral;

import org.littletonrobotics.junction.AutoLog;

public interface CorralIO {
    @AutoLog
    public static class CorralIOInputs {
        public double motorSpeed = 0.0;
        public boolean motorRunning = false;
    }

    default void updateInputs(CorralIOInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setVoltage(double volts) {}

    /** Stops the corral motor. */
    default void stop() {

        // Implementation for setting voltage
        double volts = 0.0; // Default voltage value
        System.out.println("Setting voltage to: " + volts);
        /** Stops the corral motor. */
    }
}
