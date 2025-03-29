package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOReal implements ElevatorIO {
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.ELEVATOR_L_CAN_ID, SparkMax.MotorType.kBrushless);
    private final SparkMax rightMotor =
            new SparkMax(ElevatorConstants.ELEVATOR_R_CAN_ID, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder encoder = rightMotor.getEncoder();
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LOWER_LIMIT_SWITCH_PORT);

    public ElevatorIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMITS)
                .inverted(true);
        config.softLimit
                .forwardSoftLimit(ElevatorConstants.MAX_POS)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimit(ElevatorConstants.MIN_POS)
                .reverseSoftLimitEnabled(false);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = getPosition();
        inputs.limitSwitch = !limitSwitch.get();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
