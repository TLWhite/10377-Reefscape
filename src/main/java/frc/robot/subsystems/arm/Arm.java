package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();
    private final PIDController armController = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);
    private boolean initialized = false;

    private final ShuffleboardTab tab = Shuffleboard.getTab("arm");
    private final GenericEntry positionEntry = tab.add("armValue", 0).getEntry();
    private final GenericEntry setpointEntry = tab.add("armSetpoint", 0).getEntry();

    public Arm(ArmIO io) {
        this.io = io;
        armController.setTolerance(ArmConstants.PID_TOLERANCE);
        armController.setSetpoint(0); // defer actual value until periodic
    }

    @Override
    public void periodic() {
        if (!initialized) {
            armController.setSetpoint(getPosition());
            initialized = true;
        }
        io.updateInputs(inputs);
        positionEntry.setDouble(inputs.position);
        setpointEntry.setDouble(armController.getSetpoint());
        SmartDashboard.putBoolean("arm limit", inputs.limitSwitch);
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean atSetpoint() {
        return armController.atSetpoint();
    }

    private double limit(double power) {
        if ((inputs.position < ArmConstants.MAX_POS && inputs.position > 0.315 && power > 0)
                || (inputs.position >= ArmConstants.MAX_POS || inputs.position < ArmConstants.MIN_POS) && power < 0) {
            return 0;
        }
        return power;
    }

    public void setVoltage(double volts) {
        double limitedVolts = limit(volts);
        double kfVolts = Math.sin(inputs.position * 2 * Math.PI) * ArmConstants.KF_COEFFICIENT * 12.0;
        io.setVoltage(limitedVolts + kfVolts);
        SmartDashboard.putNumber("arm axis", volts / 12.0);
        SmartDashboard.putNumber("Speed", (limitedVolts + kfVolts) / 12.0);
    }

    public void stopMotor() {
        setVoltage(0);
    }

    public void setPID(double setpoint) {
        armController.setSetpoint(setpoint);
    }

    public void executePID() {
        double pidOutput = armController.calculate(inputs.position);
        setVoltage(pidOutput);
    }

    public Trigger armLimiter() {
        return new Trigger(() -> inputs.position <= 0.09); // TODO: Tune
    }

    public Trigger armIntake() {
        return new Trigger(() -> inputs.position >= 0.42); // TODO: Tune
    }

    // Commands
    public Command armPIDCommandDefault(BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {}, // initialize
                this::executePID, // run PID loop
                interrupted -> {
                    setVoltage(0); // stop the motor when done
                },
                () -> atSetpoint(), // finish when at target
                this);
    }

    public Command manualArm(DoubleSupplier joystick, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {}, // initialize
                () -> {
                    if (canFold.getAsBoolean()) {
                        setVoltage(joystick.getAsDouble() * 0.3 * 12.0);
                    } else {
                        setVoltage(0);
                    }
                },
                interrupted -> {
                    setVoltage(0);
                    setPID(getPosition()); // hold position when released
                },
                () -> false, // never ends automatically
                this);
    }

    private Command armCommandFactory(BooleanSupplier canFold, double position) {
        return new FunctionalCommand(
                () -> {
                    if (canFold.getAsBoolean()) setPID(position);
                },
                () -> {
                    if (canFold.getAsBoolean()) executePID();
                    else setVoltage(0);
                },
                interrupted -> {
                    setPID(getPosition());
                    setVoltage(0);
                },
                () -> atSetpoint(),
                this);
    }

    public Command startArmCommand() {
        return runOnce(() -> setPID(getPosition()));
    }

    public Command armSafety(BooleanSupplier canFold) {
        return armCommandFactory(canFold, 0.25);
    }

    public Command armL0(BooleanSupplier canFold) {
        return armCommandFactory(canFold, ArmConstants.L0_POS);
    }

    public Command armL1(BooleanSupplier canFold) {
        return armCommandFactory(canFold, ArmConstants.L1_POS);
    }

    public Command armL2(BooleanSupplier canFold) {
        return armCommandFactory(canFold, ArmConstants.L2_POS);
    }

    public Command armL3(BooleanSupplier canFold) {
        return armCommandFactory(canFold, ArmConstants.L3_POS);
    }

    // Duplicate method removed

    public Command armL4(BooleanSupplier canFold) {
        return armCommandFactory(canFold, ArmConstants.L4_POS);
    }

    public Command exitState(BooleanSupplier canFold) {
        return armCommandFactory(canFold, getPosition());
    }

    public boolean isInSafeFoldRange() {
        double pos = getPosition();
        return pos >= ArmConstants.MIN_SAFE_FOLD_POS && pos <= ArmConstants.MAX_SAFE_FOLD_POS;
    }
}
