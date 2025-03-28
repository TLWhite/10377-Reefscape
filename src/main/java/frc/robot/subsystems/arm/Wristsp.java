// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.armConstants;

public class armsp extends SubsystemBase {
  /**
   * Creates a new arm subsystem. This class controls the arm motor and
   * allows for various actions like manual control, setting arm positions,
   * and running PID-based control to maintain precise arm positioning.
   */

  // CTRE hardware objects for controlling the arm motor and reading arm
  // position
  //private final TalonFX armMotor = new TalonFX(Constants.armConstants.armMotor_ID);
  // A TalonFX motor controller for the arm motor, used to control the arm's
  // movement based on power input
  //public final CANcoder armAbsEncoder = new CANcoder(Constants.armConstants.armAbsEncoder_ID);
  // A CANcoder sensor that provides the current position of the arm, which is
  // crucial for feedback control
  // Motor controller for the arm system
  private final TalonFX armMotor = new TalonFX(ArmConstants.armMotor_ID);

 private final DutyCycle armEncoder= new DutyCycle(ArmConstants.encoderPort,1,0);
 

  // Configuration objects for motor controllers

  private final DigitalInput limit = new DigitalInput(4);
  
  double armSpeed = 0.3;
  boolean armSafe = false;
  boolean safeFold = false;
  double MotorPower = 0;
  // Shuffleboard setup to display arm data for debugging and tuning
  private final PIDController armController = new PIDController(3.4, 0, 0);// TODO tune for mechanism
  // PID controller to maintain arm position by calculating the appropriate
  // motor output

  private ShuffleboardTab DS_armTab = Shuffleboard.getTab("arm");
  // Create a new tab on the Shuffleboard for arm data visualization
  private GenericEntry DS_armPosition = DS_armTab.add("armValue",
      armSpeed).getEntry();

  private GenericEntry DS_armSetpoint = DS_armTab.add("armSetpoint",
      armController.getSetpoint()).getEntry();

  // Add a speed slider to adjust the maximum arm speed in real-time through
  // Shuffleboard

  // Retrieve the arm speed from the Shuffleboard, defaulting to 0.4 if not set

  public armsp() {
   
    armController.setTolerance(.0075);

    // armSpeed = DS_armSpeed.getDouble(0.3);

    // Set the tolerance for the PID controller (how close the motor has to be to
    // the setpoint to stop adjusting)
  }
   // Configure the SparkMax motor controllers
  

  // Method to set power to the arm motor directly
  public void setPower(TalonFX motor, double power) {
    motor.set(power);
    // Set the motor power using the TalonFX motor controller
  }

  // Method to set arm motor power, limiting the power to prevent over-travel
  public void setMotor(double power) {
    double output=0;
    double limitedSpeed = Limit(power);
    
     
     output = Math.sin(getarmPosition() * 2 * Math.PI ) * .02 + limitedSpeed; // TODO tune for KF to aviod droopy arm because of gravity
    
    

    SmartDashboard.putNumber("arm axis",power);
    SmartDashboard.putNumber("Speed",Math.sin(getarmPosition() * 2 * Math.PI ) * .06 + power);

    setPower(armMotor, Limit(output));
    // Apply the arm power limit to ensure the arm doesn't exceed its mechanical
    // limits
  }

  // Method to stop the arm motor by setting the power to zero
  public void stopMotor() {
    this.setMotor(0);
    // Set the motor power to zero to stop the arm from moving
  }

  // Method to limit arm motor power based on the current arm position to
  // avoid damaging the arm
  private double Limit(double speed) {
    double output= speed;

    if((armAbsEncoder.getPosition()< 0.7&&armAbsEncoder.getPosition()> .315) && speed>0)// TODO find arm limits 
    {
      output = 0;
    }
    else if((armAbsEncoder.getPosition()>= 0.7||armAbsEncoder.getPosition() < .109) && speed<0)
    {
      output = 0;
    }
    return output;
  }

  // Trigger to limit arm movement if the arm is too low (below position 0.09)
  public Trigger armLimiter() {
    return new Trigger(() -> this.getarmPosition() <= .09);//TODO tune for mechanisim 
    // Create a trigger that activates when the arm position is below 0.09
  }

  // Trigger to activate intake if the arm is at L1 (above or equal to position
  // 0.42)
  public Trigger armIntake() {

    return new Trigger(() -> this.getarmPosition() >= .42); // TODO tune for mechanisim
    // Create a trigger that activates when the arm position is above or equal to
    // 0.42
  }

  // Method to retrieve the current arm position from the CANcoder sensor
  public double getarmPosition() {
    return (armEncoder.get());
    // Return the current arm position as a double value from the CANcoder sensor
  }

  // Method to set the desired target arm position for PID control
  public void setarmPID(double setPoint) {
    this.armController.setSetpoint(setPoint);
    // Set the desired position (setpoint) for the arm using the PID controller
  }

  // Method to execute PID control to adjust the motor power and move the arm
  // towards the target position
  public void executearmPID() {
    setMotor((this.armController.calculate(this.getarmPosition())));
    // Calculate the appropriate motor power based on the PID controller and apply
    // it to the arm motor
  }

  // Method to check if the arm has reached its target position based on the PID
  // tolerance
  public boolean armAtSetpoint() {
    return this.armController.atSetpoint();
    // Return true if the arm has reached its target position, within the
    // specified tolerance (0.01)
  }

 

  // Command to run the arm motor with PID control
  public Command armPIDCommandDefault(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.safeFold = canFold.getAsBoolean();
        }, // Initialize: No action needed
        () -> {
          this.executearmPID();
          this.safeFold = canFold.getAsBoolean();
        }, // Execute: Run the PID control to adjust arm position
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> {
          return false; // Finish condition: Always false, meaning the command never finishes
                        // automatically
        },
        this); // Subsystem: This command is bound to the arm subsystem
  }

  // Command for manual control of the arm motor, using joystick input for arm
  // movement
  public Command Manualarm(DoubleSupplier armJoystick, BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.safeFold = canFold.getAsBoolean();
          
        }, // Initialize: No action needed
        () -> {
          // this.armSpeed = this.DS_armSpeed.getDouble(armSpeed);
          this.setarmMotor(armJoystick.getAsDouble() * 0.3);
        }, // Execute: Set arm motor power based
        // this.armSpeed
        // on joystick
        // input (scaled by 0.2 for control)
        interrupted -> {
          this.setarmMotor(0);
          this.setarmPID(getarmPosition());}, // Interrupted: Reset to the current arm position if
                                                             // interrupted
        () -> {
          return false; // Finish condition: Always false, meaning the command never finishes
                        // automatically
        },
        this); // Subsystem: This command is bound to the arm subsystem
  }

  



  public Command armCommandFactory(BooleanSupplier canFold, double position) {
    return new FunctionalCommand(
        () -> {
          this.setarmPID(position);
          this.safeFold = canFold.getAsBoolean(); // Set arm to position L1 (0.445)
        },
        () -> {
          this.safeFold = canFold.getAsBoolean();
          this.executearmPID();

        },
        interrupted -> {
          this.setarmPID(this.getarmPosition());
          this.setarmMotor(0);

        }, // Interrupted: No specific action when interrupted
        () ->this.armAtSetpoint(), // Finish condition: Check if arm has reached L1 position
        this);
  }

  // Command to start the arm motor but with no action (used for state
  // transitions)
  public Command startarmCommand() { // This command makes the arm hold its starting position
    return this.runOnce(() -> {
      this.setarmPID(this.getarmPosition());
    });
  }

  // Command to ensure the arm is moved to a safe position
  public Command armSafety(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.25);//.until(() -> this.getarmPosition() < 0.15); // this command makes
    // sure the arm is in an orientation that can't crash
  }

  public Command armL0(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.2);
    
  }
 /*  public Command armAlgeaIntake(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.13);
    
  } */
  // Commands for moving the arm to various predefined positions (L1, L2, L3,
  // L4) to edit these positions change the number linked to the position inside
  // armCommandFactory
  public Command armL1(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.25);//.25 is orginal
    // Subsystem: This command is bound to the arm subsystem
  }

  public Command armL2(BooleanSupplier canFold) {
    return armCommandFactory(canFold, .179);

  }

  public Command armL3(BooleanSupplier canFold) {
    return armCommandFactory(canFold, .1365);
    // Subsystem: This command is bound to the arm subsystem
  }

  public Command armL4(BooleanSupplier canFold) {
    return armCommandFactory(canFold, .0887);
    // Subsystem: This command is bound to the arm subsystem
  }

  /* public Command armA1(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.2);

  }

  public Command armA2(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.210
    );
    // Subsystem: This command is bound to the arm subsystem
  }

  public Command armBarge(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.289); // Potentially .308
    // Subsystem: This command is bound to the arm subsystem
  }

  public Command armProcessor(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.2);
    // Subsystem: This command is bound to the arm subsystem
  }

  public Command armClimber(BooleanSupplier canFold) {
    return armCommandFactory(canFold, 0.4);
    // Subsystem: This command is bound to the arm subsystem
  } */

  // Command to exit arm state, keeping it in its current position
  public Command ExitState(BooleanSupplier canFold) {
    return armCommandFactory(canFold, this.getarmPosition());

  }

  // Periodic method called once per scheduler run, used for updating real-time
  // data
  @Override
  public void periodic() {

    this.DS_armPosition.setDouble(getarmPosition()); // Update the arm's current position on Shuffleboard
    // Update arm motor speed based on Shuffleboard slider
    // input
    // this.armSpeed = this.DS_armSpeed.getDouble(armSpeed);
    this.DS_armSetpoint.setDouble(armController.getSetpoint());
    
    SmartDashboard.putBoolean("arm limit", limit.get());
    // This method runs periodically to keep the arm data current and allow for
    // real-time adjustments
  }
}