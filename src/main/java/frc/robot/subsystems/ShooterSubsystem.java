// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends PIDSubsystem {
  
  WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_CHANNEL);



  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(double kP, double kI, double kD, double kF) {
    super(
        // The PIDController used by the subsystem
        new PIDController(kP, kI, kD));

    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
    ShooterConstants.PID_LOOPTYPE,
    ShooterConstants.TIMEOUT_MS);
    configPIDF(shooterMotor, kP, kI, kD, kF);

  }

  public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
    motorController.config_kP(ShooterConstants.SLOT_ID, P);
    motorController.config_kI(ShooterConstants.SLOT_ID, I);
    motorController.config_kD(ShooterConstants.SLOT_ID, D);
    motorController.config_kF(ShooterConstants.SLOT_ID, F);
  }

  /**
   * Sets the speed of shooter
   * @param speed percent output [-1, 1]
   */
  public void shooterManual(double speed) {
    shooterMotor.set(speed);
  }

  /**
   * Sets rotations per minute
   * @param rpm input of rotations per minute
   */
  public void shooterPID(double rpm) {
    
    rpm = MathUtil.clamp(rpm, 0, ShooterConstants.MAX_SPEED_RPM);

    shooterMotor.set(ControlMode.Velocity, rpmToTicksPer100ms(rpm));
  }

  /**
   * Converts rotations per minute to ticks per 100ms
   * Multiplies rotations per minute by ticks per rotation
   * Divides by 600 to convert from minute to millisecond
   * @param rpm input of rotations per minute
   * @return output of ticks per 100ms
   */
  public double rpmToTicksPer100ms(double rpm) {
    return rpm * ShooterConstants.TICKS_PER_ROTATION / 600;
  }

  /**
   * Resets shooter encoder
   */
  public void resetEncoder() {
    shooterMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
