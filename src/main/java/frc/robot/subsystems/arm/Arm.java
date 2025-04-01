package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();
    private final PIDController armController = new PIDController(ArmConstants.P, ArmConstants.I, ArmConstants.D);
    private boolean initialized = false;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    private final GenericEntry positionEntry = tab.add("Arm Position", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 2)
            .withPosition(6, 4)
            .getEntry();
    private final GenericEntry setpointEntry = tab.add("Target Setpoint", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .getEntry();

    private final GenericEntry kPEntry =
            tab.add("kP", ArmConstants.P).withPosition(4, 0).getEntry();
    private final GenericEntry kIEntry =
            tab.add("kI", ArmConstants.I).withPosition(4, 1).getEntry();
    private final GenericEntry kDEntry =
            tab.add("kD", ArmConstants.D).withPosition(4, 2).getEntry();
    private final GenericEntry inputSetpoint =
            tab.add("Manual Setpoint Input", 0.0).withPosition(0, 1).getEntry();

    private final GenericEntry canLiftIndicator = tab.add("Can Lift", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "Lime"))
            .withPosition(6, 1)
            .getEntry();

    private final GenericEntry encoderPosition =
            tab.add("Abs Encoder Pos", 0.0).withPosition(0, 5).getEntry();
    private final GenericEntry encoderOffset =
            tab.add("Encoder Offset", 0.0).withPosition(0, 6).getEntry();
    private final GenericEntry armAngleIndicator =
            tab.add("Arm Angle (deg)", 0.0).withPosition(0, 7).getEntry();

    private final GenericEntry l4Entry =
            tab.add("Setpoint L4", ArmConstants.L4_POS).withPosition(0, 2).getEntry();
    private final GenericEntry l3Entry =
            tab.add("Setpoint L3", ArmConstants.L3_POS).withPosition(0, 3).getEntry();
    private final GenericEntry l2Entry =
            tab.add("Setpoint L2", ArmConstants.L2_POS).withPosition(0, 4).getEntry();
    private final GenericEntry l1Entry =
            tab.add("Setpoint L1", ArmConstants.L1_POS).withPosition(0, 5).getEntry();
    private final GenericEntry l0Entry =
            tab.add("Setpoint L0", ArmConstants.L0_POS).withPosition(0, 6).getEntry();
    private final GenericEntry loadEntry =
            tab.add("Setpoint Load", ArmConstants.LOAD_POS).withPosition(0, 7).getEntry();

    public Arm(ArmIO io) {
        this.io = io;
        armController.setTolerance(ArmConstants.PID_TOLERANCE);
        armController.setSetpoint(0.0);

        tab.add("Update PID", new InstantCommand(this::resetPID, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 0)
                .withProperties(Map.of("Label", "Apply PID Values"));

        tab.add("Move to Manual", new InstantCommand(this::applySetpoint, this))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 1)
                .withProperties(Map.of("Label", "Move to Manual Setpoint"));

        tab.add("Move to L4", new InstantCommand(() -> setPID(l4Entry.getDouble(ArmConstants.L4_POS)), this))
                .withPosition(1, 2);
        tab.add("Move to L3", new InstantCommand(() -> setPID(l3Entry.getDouble(ArmConstants.L3_POS)), this))
                .withPosition(1, 3);
        tab.add("Move to L2", new InstantCommand(() -> setPID(l2Entry.getDouble(ArmConstants.L2_POS)), this))
                .withPosition(1, 4);
        tab.add("Move to L1", new InstantCommand(() -> setPID(l1Entry.getDouble(ArmConstants.L1_POS)), this))
                .withPosition(1, 5);
        tab.add("Move to L0", new InstantCommand(() -> setPID(l0Entry.getDouble(ArmConstants.L0_POS)), this))
                .withPosition(1, 6);
        tab.add("Move to Load", new InstantCommand(() -> setPID(loadEntry.getDouble(ArmConstants.LOAD_POS)), this))
                .withPosition(1, 7);
    }

    @Override
    public void periodic() {
        if (!initialized) {
            setPID(0.0);
            initialized = true;
        }

        io.updateInputs(inputs);
        positionEntry.setDouble(inputs.position);
        setpointEntry.setDouble(armController.getSetpoint());
        canLiftIndicator.setBoolean(isInSafeFoldRange());

        encoderPosition.setDouble(inputs.position);
        encoderOffset.setDouble(inputs.offset);
        armAngleIndicator.setDouble((inputs.position - inputs.offset) * 360);

        SmartDashboard.putBoolean("arm limit", inputs.limitSwitch);
    }

    public void setVoltage(double volts) {
        double kfVolts = Math.sin(inputs.position * 2 * Math.PI) * ArmConstants.KF_COEFFICIENT * 12.0;
        io.setVoltage(volts + kfVolts);
    }

    public void stopMotor() {
        setVoltage(0.0);
    }

    public void resetPID() {
        armController.setPID(
                kPEntry.getDouble(ArmConstants.P),
                kIEntry.getDouble(ArmConstants.I),
                kDEntry.getDouble(ArmConstants.D));
    }

    public void applySetpoint() {
        setPID(inputSetpoint.getDouble(0.0));
    }

    public void setPID(double setpoint) {
        armController.setSetpoint(setpoint);
    }

    public void executePID() {
        double output = armController.calculate(inputs.position);
        setVoltage(output);
    }

    public boolean atSetpoint() {
        return armController.atSetpoint();
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean isInSafeFoldRange() {
        return inputs.position >= ArmConstants.MIN_SAFE_FOLD_POS && inputs.position <= ArmConstants.MAX_SAFE_FOLD_POS;
    }

    public Command armL0(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L0_POS, canFold);
    }

    public Command armL1(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L1_POS, canFold);
    }

    public Command armL2(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L2_POS, canFold);
    }

    public Command armL3(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L3_POS, canFold);
    }

    public Command armL4(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.L4_POS, canFold);
    }

    public Command armLoad(BooleanSupplier canFold) {
        return createArmCommand(ArmConstants.LOAD_POS, canFold);
    }

    public Command manualArm(DoubleSupplier joystick, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> {},
                () -> setVoltage(canFold.getAsBoolean() ? joystick.getAsDouble() * 3.0 : 0.0),
                interrupted -> stopMotor(),
                () -> false,
                this);
    }

    private Command createArmCommand(double target, BooleanSupplier canFold) {
        return new FunctionalCommand(
                () -> setPID(target),
                () -> {
                    if (canFold.getAsBoolean()) executePID();
                    else stopMotor();
                },
                interrupted -> stopMotor(),
                this::atSetpoint,
                this);
    }
}
