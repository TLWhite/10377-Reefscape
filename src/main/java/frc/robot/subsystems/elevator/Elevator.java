package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    private Arm arm;

    public void setArm(Arm arm) {
        this.arm = arm;
    }

    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    private final PIDController elevatorController =
            new PIDController(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D);

    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private final GenericEntry positionEntry = tab.add("ElevatorValue", 0).getEntry();
    private final GenericEntry setpointEntry =
            tab.add("Setpoint", elevatorController.getSetpoint()).getEntry();

    public Elevator(ElevatorIO io) {
        this.io = io;
        elevatorController.setTolerance(ElevatorConstants.PID_TOLERANCE);
        elevatorController.setSetpoint(ElevatorConstants.LOAD_POS); // Use safe default
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        positionEntry.setDouble(inputs.position);
        setpointEntry.setDouble(elevatorController.getSetpoint());
        SmartDashboard.putBoolean("elevator limit", inputs.limitSwitch);
        if (inputs.limitSwitch && inputs.position != 0) {
            zeroElevator();
        }
        SmartDashboard.putData(this);
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean atSetpoint() {
        return elevatorController.atSetpoint();
    }

    private double limitPower(double power) {
        if (arm != null && !arm.isInSafeFoldRange() && power > 0 && inputs.position < 50) {
            return 0.0;
        }
        if ((inputs.position >= ElevatorConstants.MAX_POS && power > 0)
                || (inputs.position <= ElevatorConstants.MIN_POS && power < 0)) {
            return 0.0;
        }
        return power;
    }

    private double getKF() {
        return inputs.position < 0.5 ? ElevatorConstants.KF_BELOW_0_5 : ElevatorConstants.KF_ABOVE_0_5;
    }

    public void setVoltage(double volts) {
        double limitedVolts = limitPower(volts);
        double kfVolts = getKF() * 12.0;
        io.setVoltage(limitedVolts + kfVolts);
        SmartDashboard.putNumber("Elevator Output", (limitedVolts + kfVolts) / 12.0);
        SmartDashboard.putNumber("Elevator Position", inputs.position);
        SmartDashboard.putNumber("Elevator Setpoint", elevatorController.getSetpoint());
        SmartDashboard.putBoolean("Arm in Safe Fold Range", arm == null || arm.isInSafeFoldRange());
        SmartDashboard.putString(
                "Elevator Status", arm == null ? "No Arm Set" : arm.isInSafeFoldRange() ? "Safe" : "Unsafe");
    }

    public void setPID(double setpoint) {
        elevatorController.setSetpoint(setpoint);
    }

    public void executePID() {
        double pidOutput = elevatorController.calculate(inputs.position);
        setVoltage(pidOutput);
    }

    public void zeroElevator() {
        io.setPosition(0);
    }

    public double getThrottle() {
        return 1 - 0.5 * (inputs.position / ElevatorConstants.THROTTLE_DIVISOR);
    }

    public BooleanSupplier canFold() {
        return () -> arm != null && arm.isInSafeFoldRange();
    }

    public Command manualElevator(DoubleSupplier joystick) {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(joystick.getAsDouble() * 0.8 * 12.0),
                interrupted -> setPID(getPosition()),
                () -> false,
                this);
    }

    public Command startCommand() {
        return runOnce(() -> setPID(getPosition()));
    }

    public Command pidCommandDefault() {
        return new FunctionalCommand(() -> {}, this::executePID, interrupted -> {}, () -> false, this);
    }

    public Command moveToPosition(double position) {
        return new FunctionalCommand(
                () -> setPID(position),
                this::executePID,
                interrupted -> {
                    setPID(getPosition());
                    setVoltage(0);
                },
                this::atSetpoint,
                this);
    }

    public Command elevatorLoad() {
        return moveToPosition(ElevatorConstants.LOAD_POS);
    }

    public Command elevatorL1() {
        return moveToPosition(ElevatorConstants.L1_POS);
    }

    public Command elevatorL2() {
        return moveToPosition(ElevatorConstants.L2_POS);
    }

    public Command elevatorL3() {
        return moveToPosition(ElevatorConstants.L3_POS);
    }

    public Command elevatorL4() {
        return moveToPosition(ElevatorConstants.L4_POS);
    }

    public Command exitState() {
        return moveToPosition(getPosition());
    }
}
