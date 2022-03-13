// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  public WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.FLYWHEEL_CHANNEL);
  //public WPI_TalonFX hoodMotor = new WPI_TalonFX(ShooterConstants.HOOD_CHANNEL);
  private boolean isRunning = false;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    configFeedbackSensors();

    // kF calculation found here: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
    configPIDF(shooterMotor, ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D, 1023 / rpmToTicksPer100ms(ShooterConstants.MAX_SPEED_RPM));

    shooterMotor.configClosedLoopPeakOutput(0, 1);

    shooterMotor.setNeutralMode(NeutralMode.Coast);

    shooterMotor.setInverted(true);

  }

  public boolean getIsRunning() {
    return isRunning;
  }

  public void setIsRunning(boolean set) {
    isRunning = set;
  }
  
  private void configFeedbackSensors() {
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ShooterConstants.PID_LOOPTYPE, ShooterConstants.TIMEOUT_MS);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder velocity", ticksToRPM(shooterMotor.getSelectedSensorVelocity()));
    // SmartDashboard.putNumber("Turret Encoder Error", shooterMotor.getClosedLoopError());

  }

  public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
    motorController.config_kP(ShooterConstants.SLOT_ID, P);
    motorController.config_kI(ShooterConstants.SLOT_ID, I);
    motorController.config_kD(ShooterConstants.SLOT_ID, D);
    motorController.config_kF(ShooterConstants.SLOT_ID, F);
  }

  public void flywheelToggle(double rpm) {
    // if (!isRunning) {
    //   flywheelPID(rpm);
    //   isRunning = true;
    // } else {
    //   flywheelManual(0);
    //   isRunning = false;
    // }

    if (!isRunning) {
      flywheelManual(rpm);
      isRunning = true;
    } else {
      flywheelManual(0);
      isRunning = false;
    }


    
  }

  /**
   * Sets the speed of shooter
   * @param speed percent output [-1, 1]
   */
  public void flywheelManual(double speed) {
    shooterMotor.set(speed);
  }

  /**
   * Sets rotations per minute
   * @param rpm input of rotations per minute
   */
  public void flywheelPID(double rpm) {
    
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

  public double ticksToRPM(double tick) {
    return (tick * 600) / ShooterConstants.TICKS_PER_ROTATION;
  }

  /**
   * Check if the Shooter is at the correct Rotations per minute (rpm)
   *
   * @param targetRPM The target rpm
   * @return true if the Shooter is at the specified rpm, false otherwise
   */
  public boolean flywheelAtTargetRPM(double targetRPM) {
    return Math.abs(targetRPM - convertCPDToRPM(shooterMotor.getSelectedSensorVelocity()))
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
      shooterMotor.set(0);
  }

  public double getFlywheelVelocity() {
    return convertCPDToRPM(shooterMotor.getSelectedSensorVelocity());
  }

}
