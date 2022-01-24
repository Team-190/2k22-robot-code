// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  public WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(ShooterConstants.BOTTOM_SHOOTER_CHANNEL);
  public WPI_TalonFX topShooterMotor = new WPI_TalonFX(ShooterConstants.TOP_SHOOTER_CHANNEL);


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    configFeedbackSensors();
    configPIDF(bottomShooterMotor, ShooterConstants.BOTTOM_P, ShooterConstants.BOTTOM_I, ShooterConstants.BOTTOM_D, ShooterConstants.BOTTOM_F);
    configPIDF(topShooterMotor, ShooterConstants.TOP_P, ShooterConstants.TOP_I, ShooterConstants.TOP_D, ShooterConstants.TOP_F);

    bottomShooterMotor.configClosedLoopPeakOutput(0, 1);
    topShooterMotor.configClosedLoopPeakOutput(0, 1);

    bottomShooterMotor.setNeutralMode(NeutralMode.Coast);
    topShooterMotor.setNeutralMode(NeutralMode.Coast);

    bottomShooterMotor.setInverted(true);

  }
  
  private void configFeedbackSensors() {
    bottomShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ShooterConstants.PID_LOOPTYPE, ShooterConstants.TIMEOUT_MS);
    topShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ShooterConstants.PID_LOOPTYPE, ShooterConstants.TIMEOUT_MS);
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
  public void bottomShooterManual(double speed) {
    bottomShooterMotor.set(speed);
  }

  /**
   * Sets rotations per minute
   * @param rpm input of rotations per minute
   */
  public void bottomshooterPID(double rpm) {
    
    rpm = MathUtil.clamp(rpm, 0, ShooterConstants.MAX_SPEED_RPM);

    bottomShooterMotor.set(ControlMode.Velocity, rpmToTicksPer100ms(rpm));
  }

  /**
   * Sets the speed of shooter
   * @param speed percent output [-1, 1]
   */
  public void topShooterManual(double speed) {
    topShooterMotor.set(speed);
  }

  /**
   * Sets rotations per minute
   * @param rpm input of rotations per minute
   */
  public void topShooterPID(double rpm) {

    rpm = MathUtil.clamp(rpm, 0, ShooterConstants.MAX_SPEED_RPM);

    topShooterMotor.set(ControlMode.Velocity, rpmToTicksPer100ms(rpm));
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
   * Check if the Shooter is at the correct Rotations per minute (rpm)
   *
   * @param targetRPM The target rpm
   * @return true if the Shooter is at the specified rpm, false otherwise
   */
  public boolean bottomAtTargetRPM(double targetRPM) {
    return Math.abs(targetRPM - convertCPDToRPM(bottomShooterMotor.getSelectedSensorVelocity()))
            < Constants.ShooterConstants.RPM_THRESHOLD;
  }
  /**
   * Check if the Shooter is at the correct Rotations per minute (rpm)
   *
   * @param targetRPM The target rpm
   * @return true if the Shooter is at the specified rpm, false otherwise
   */
  public boolean topAtTargetRPM(double targetRPM) {
    return Math.abs(targetRPM - convertCPDToRPM(topShooterMotor.getSelectedSensorVelocity()))
            < Constants.ShooterConstants.RPM_THRESHOLD;
  }

  /**
   * Converts a value from Counts per Diameter to Rotations per Minute
   *
   * @param cpd the cpd to convert
   * @return the rpm of that cpd
   */
  public double convertCPDToRPM(double cpd) {
    return cpd
            * (1.0 / Constants.ShooterConstants.TICKS_PER_ROTATION)
            * 60.0
            * 10.0;
  }

  /**
   * Converts a value from Rotations per Minute to Counts per Diameter
   *
   * @param rpm the rpm to convert
   * @return the cpd of that rpm
   */
  public static double convertRPMToCPD(double rpm) {
    return rpm
            * ShooterConstants.TICKS_PER_ROTATION
            * (1.0 / 60.0)
            * (1.0 / 10.0);
  }

  public void stop() {
      bottomShooterMotor.set(0);
      topShooterMotor.set(0);
  }

  public double getBottomVelocity() {
    return convertCPDToRPM(bottomShooterMotor.getSelectedSensorVelocity());
  }

  public double getTopVelocity() {
    return convertCPDToRPM(topShooterMotor.getSelectedSensorVelocity());
  }
}
