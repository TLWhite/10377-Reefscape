package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();
    private final PIDController armController = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);
    private boolean initialized = false;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    private final GenericEntry positionEntry =
            tab.add("Arm Position", 0).withPosition(0, 0).getEntry();
    private final GenericEntry setpointEntry =
            tab.add("Arm Setpoint", 0).withPosition(1, 0).getEntry();

    private final GenericEntry kPEntry = tab.add("kP", Preferences.getDouble("Arm_kP", ArmConstants.P))
            .withPosition(0, 1)
            .getEntry();
    private final GenericEntry kIEntry = tab.add("kI", Preferences.getDouble("Arm_kI", ArmConstants.I))
            .withPosition(1, 1)
            .getEntry();
    private final GenericEntry kDEntry = tab.add("kD", Preferences.getDouble("Arm_kD", ArmConstants.D))
            .withPosition(2, 1)
            .getEntry();
    private final GenericEntry setpointInput =
            tab.add("Setpoint Input", 0).withPosition(3, 1).getEntry();

    private final GenericEntry l0Entry =
            tab.add("L0 Pos", ArmConstants.L0_POS).withPosition(0, 3).getEntry();
    private final GenericEntry l1Entry =
            tab.add("L1 Pos", ArmConstants.L1_POS).withPosition(1, 3).getEntry();
    private final GenericEntry l2Entry =
            tab.add("L2 Pos", ArmConstants.L2_POS).withPosition(2, 3).getEntry();
    private final GenericEntry l3Entry =
            tab.add("L3 Pos", ArmConstants.L3_POS).withPosition(3, 3).getEntry();
    private final GenericEntry l4Entry =
            tab.add("L4 Pos", ArmConstants.L4_POS).withPosition(4, 3).getEntry();

    public Arm(ArmIO io) {
        this.io = io;
        armController.setTolerance(ArmConstants.PID_TOLERANCE);

        tab.add("Apply Setpoint", new InstantCommand(this::applySetpoint, this))
                .withPosition(4, 0)
                .withSize(1, 1);
        tab.add("Update PID", new InstantCommand(this::resetPID, this))
                .withPosition(3, 0)
                .withSize(1, 1);

        tab.add("Go L0", new InstantCommand(() -> setPID(l0Entry.getDouble(ArmConstants.L0_POS)), this))
                .withPosition(0, 4);
        tab.add("Go L1", new InstantCommand(() -> setPID(l1Entry.getDouble(ArmConstants.L1_POS)), this))
                .withPosition(1, 4);
        tab.add("Go L2", new InstantCommand(() -> setPID(l2Entry.getDouble(ArmConstants.L2_POS)), this))
                .withPosition(2, 4);
        tab.add("Go L3", new InstantCommand(() -> setPID(l3Entry.getDouble(ArmConstants.L3_POS)), this))
                .withPosition(3, 4);
        tab.add("Go L4", new InstantCommand(() -> setPID(l4Entry.getDouble(ArmConstants.L4_POS)), this))
                .withPosition(4, 4);
    }

    private void resetPID() {
        double p = kPEntry.getDouble(ArmConstants.P);
        double i = kIEntry.getDouble(ArmConstants.I);
        double d = kDEntry.getDouble(ArmConstants.D);
        armController.setPID(p, i, d);
        Preferences.setDouble("Arm_kP", p);
        Preferences.setDouble("Arm_kI", i);
        Preferences.setDouble("Arm_kD", d);
    }

    private void applySetpoint() {
        double setpoint = setpointInput.getDouble(0);
        armController.setSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        if (!initialized) {
            armController.setSetpoint(-0.05);
            initialized = true;
        }

        io.updateInputs(inputs);
        positionEntry.setDouble(inputs.position);
        setpointEntry.setDouble(armController.getSetpoint());
        SmartDashboard.putBoolean("At Setpoint", atSetpoint());
        SmartDashboard.putBoolean("canLift", isInSafeFoldRange());
    }

    public void zeroEncoder() {
        io.zeroEncoder();
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean atSetpoint() {
        return armController.atSetpoint();
    }

    private double limit(double power) {
        if ((inputs.position > ArmConstants.MAX_POS && power > 0)
                || (inputs.position < ArmConstants.MIN_POS && power > 0)) {
            return 0;
        }
        return power;
    }

    public void setVoltage(double volts) {
        double limitedVolts = limit(volts);
        double kfVolts = Math.sin(inputs.position * 2 * Math.PI) * ArmConstants.KF_COEFFICIENT * 12.0;
        io.setVoltage(limitedVolts + kfVolts);
    }

    public void stopMotor() {
        setVoltage(0);
    }

    public void setPID(double setpoint) {
        armController.setSetpoint(setpoint);
    }

    public void executePID() {
        double output = armController.calculate(inputs.position);
        setVoltage(output);
    }

    public Trigger armLimiter() {
        return new Trigger(() -> inputs.position <= 0.09);
    }

    public Trigger armIntake() {
        return new Trigger(() -> inputs.position >= 0.42);
    }

    public Command manualArm(DoubleSupplier joystick, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    if (canFold.getAsBoolean()) {
                        setVoltage(joystick.getAsDouble() * 0.3 * 12.0);
                    } else {
                        setVoltage(0);
                    }
                },
                interrupted -> {
                    stopMotor();
                    setPID(getPosition());
                },
                () -> false,
                this);
    }

    private Command armCommandFactory(BooleanSupplier canFold, double pos) {
        return new FunctionalCommand(
                () -> {
                    if (canFold.getAsBoolean()) setPID(pos);
                },
                () -> {
                    if (canFold.getAsBoolean()) executePID();
                    else stopMotor();
                },
                interrupted -> stopMotor(),
                this::atSetpoint,
                this);
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

    public Command armL4(BooleanSupplier canFold) {
        return armCommandFactory(canFold, ArmConstants.L4_POS);
    }

    public boolean isInSafeFoldRange() {
        return inputs.position >= ArmConstants.MIN_SAFE_FOLD_POS && inputs.position <= ArmConstants.MAX_SAFE_FOLD_POS;
    }
}
