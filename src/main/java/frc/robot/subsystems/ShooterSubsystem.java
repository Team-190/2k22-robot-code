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
  public WPI_TalonFX hoodMotor = new WPI_TalonFX(ShooterConstants.HOOD_CHANNEL);
  private boolean isRunning = false;
  private boolean toggle = false;
  private boolean reset = false;
  private ShooterState ShooterState;
  
  public enum ShooterState {
    Off, Short, Long
  }


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterMotor.configFactoryDefault();
    hoodMotor.configFactoryDefault();

    

    // kF calculation found here: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html
    configPIDF(
              shooterMotor, 
              ShooterConstants.FLYWHEEL_P, 
              ShooterConstants.FLYWHEEL_I, 
              ShooterConstants.FLYWHEEL_D, 
              1023 / rpmToTicksPer100ms(ShooterConstants.MAX_SPEED_RPM));

    configPIDF(
              hoodMotor, 
              ShooterConstants.HOOD_P, 
              ShooterConstants.HOOD_I, 
              ShooterConstants.HOOD_D, 
              1023 / rpmToTicksPer100ms(ShooterConstants.MAX_SPEED_RPM));
              

            
    shooterMotor.configClosedLoopPeakOutput(0, 1);
    shooterMotor.configClosedLoopPeriod(0, 1);
    shooterMotor.configClosedloopRamp(0.00);

    hoodMotor.configClosedLoopPeakOutput(0, 1);
    hoodMotor.configClosedLoopPeriod(0, 1);
    hoodMotor.configClosedloopRamp(0.00);

    hoodMotor.configAllowableClosedloopError(0, ShooterConstants.HOOD_TOLERANCE);
    shooterMotor.configAllowableClosedloopError(0, ShooterConstants.RPM_THRESHOLD);

    hoodMotor.overrideSoftLimitsEnable(false);
    hoodMotor.overrideLimitSwitchesEnable(false);

    configFeedbackSensors();

    shooterMotor.setNeutralMode(NeutralMode.Coast);
    hoodMotor.setNeutralMode(NeutralMode.Brake);
 
    shooterMotor.setInverted(true);

  }

  public boolean getIsRunning() {
    return isRunning;
  }

  public void setIsRunning(boolean set) {
    isRunning = set;
  }

  public void setToggle(boolean set) {
    toggle = set;
  }

  public boolean getToggle() {
    return toggle;
  }

  public void setShooterState(ShooterState set) {
    shooterState = set;
  }
  public ShooterState getShooterState() {
    return shooterState;
  }
  
  private void configFeedbackSensors() {
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ShooterConstants.PID_LOOPTYPE, ShooterConstants.TIMEOUT_MS);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ShooterConstants.PID_LOOPTYPE, ShooterConstants.TIMEOUT_MS);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Shooter Encoder velocity", ticksToRPM(shooterMotor.getSelectedSensorVelocity()));
    // SmartDashboard.putNumber("Hood Encoder Position", hoodMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Hood Degrees Position", hoodTicksToDegrees(hoodMotor.getSelectedSensorPosition()));
    // SmartDashboard.putBoolean("Hood Limit", getHoodLimit());
    SmartDashboard.putBoolean("ShooterRunning", getIsRunning());

    if (getHoodLimit()&&!reset) {
      resetHood(27);
      reset = true;
    } else if (!getHoodLimit()) {
      reset = false;
    }

    // SmartDashboard.putNumber("Turret Encoder Error", shooterMotor.getClosedLoopError());

  }

  public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
    motorController.config_kP(ShooterConstants.SLOT_ID, P);
    motorController.config_kI(ShooterConstants.SLOT_ID, I);
    motorController.config_kD(ShooterConstants.SLOT_ID, D);
    motorController.config_kF(ShooterConstants.SLOT_ID, F);
  }

  /**
   * Toggle the flywheel at set RPM or stop based on state
   * @param rpm 
   */
  public void flywheelToggle(double rpm) {
    if (!isRunning) {
      flywheelPID(rpm);
    } else {
      stopFlywheel();
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
    isRunning = true;

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
   * Convertes ticksPer100ms to RPM
   * @param tick input of how many encoder ticks
   * @return output of RPM
   */
  public double ticksToRPM(double tick) {
    return (tick * 600) / ShooterConstants.TICKS_PER_ROTATION;
  }

  /**
   * Check if the Shooter is at the correct Rotations per minute (rpm)
   * @return true if the Shooter is at the specified rpm, false otherwise
   */
  public boolean flywheelAtTargetRPM() {
    return (Math.abs(shooterMotor.getSelectedSensorPosition()-shooterMotor.getClosedLoopTarget())<=ShooterConstants.RPM_THRESHOLD);
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

  public double hoodTicksToDegrees(double ticks){
    return ticks/ShooterConstants.TICKS_PER_DEGREE;
  }

  public double hoodDegreesToTicks (double degrees){
    return degrees * ShooterConstants.TICKS_PER_DEGREE;
  }

  public void hoodManual(double speed) {
    hoodMotor.set(speed);
  }
  
  /**
   * Adjust hood angle based on the distance from the target
   * @param distance the distance from the vision target (in inches)
   */
  public void adjustHood(double distance){
    if (distance < 100){
      setHoodAngle(32);
    } else if (distance < 120){
      setHoodAngle(27);
    } else if (distance < 130){
      setHoodAngle(36);
    }
    else if (distance < 140){
      setHoodAngle(36);
    }
    else if (distance < 180){
      setHoodAngle(33);
    }
    else if (distance < 220){
      setHoodAngle(40);
    }
    else if (distance < 244){
      setHoodAngle(44);
    }
    else{
      setHoodAngle(52);
    }
    /*
    double minAngle = ShooterConstants.HOOD_MINIMUM_LIMIT;
    double maxAngle = ShooterConstants.HOOD_MAXIMUM_LIMIT;

    // Data from excel
    double slope = 0.0793; //m
    double yIntercept = 13.857; //b

    // Distance from target in inches
    double distanceAtMinHoodAngle = 166;
    double distanceAtMaxHoodAngle = 494;

    if (distance <= distanceAtMinHoodAngle) {
      setHoodAngle(minAngle);
    } else if (distance < distanceAtMaxHoodAngle) {
      setHoodAngle(slope*distance + yIntercept);
    } else {
      setHoodAngle(maxAngle);
    } */
  }

  public void adjustFlywheel(double distance) {
    if (distance < 100){
      flywheelPID(2200);
    } else if (distance < 120){
      flywheelPID(2300);
    } else if (distance < 130){
      flywheelPID(2150);
    }
    else if (distance < 140){
      flywheelPID(2200);
    }
    else if (distance < 180){
      flywheelPID(2500);
    }
    else if (distance < 220){
      flywheelPID(2700);
    }
    else if (distance < 244){
      flywheelPID(2750);
    }
    else{
      flywheelPID(3000);
    }
  }

  /**
   * Adjust the Flywheel RPM and the Hood Angle based on the distance from the target
   * @param distance the distance from the vision target (in inches)
   */
  public void adjustShooter(double distance) {
    if (distance < 100){
      setHoodAngle(32);
      flywheelPID(2200);
    } else if (distance < 120){
      setHoodAngle(27);
      flywheelPID(2300);
    } else if (distance < 130){
      setHoodAngle(36);
      flywheelPID(2150);
    } else if (distance < 135){
      setHoodAngle(36); // 28
      flywheelPID(2300); // 2100
    }
    else if (distance < 140){
      setHoodAngle(36); // 28
      flywheelPID(2200); // 2100
    }
    else if (distance < 180){
      setHoodAngle(33);
      flywheelPID(2500);
    }
    else if (distance < 220){
      setHoodAngle(40);
      flywheelPID(2700);
    }
    else if (distance < 244){
      setHoodAngle(44);
      flywheelPID(2750);
    }
    else{
      setHoodAngle(52);
      flywheelPID(3000);
    }

    /*double minRPM = 2200;
    double maxRPM = ShooterConstants.MAX_SPEED_RPM;

    // Data from excel
    double slope = 8.3014; //m
    double yIntercept = 2342; //b

    // Distance from target in inches
    double distanceAtMinRPM = 166;
    double distanceAtMaxRPM = 487;

    if (distance <= distanceAtMinRPM) {
      flywheelPID(2.5*distance + 1975);
    } else if (distance < distanceAtMaxRPM) {
      flywheelPID(slope*distance + yIntercept);
    } else {
      flywheelPID(maxRPM);
    }
    isRunning = true;*/
  }

  /**
   * Sets the angle of the hood
   * @param degrees the angle the hood is set to (in degrees)
   */
  public void setHoodAngle(double degrees){
    if (degrees > ShooterConstants.HOOD_MAXIMUM_LIMIT) {
      degrees = ShooterConstants.HOOD_MAXIMUM_LIMIT;
    } else if (degrees < ShooterConstants.HOOD_MINIMUM_LIMIT) {
      degrees = ShooterConstants.HOOD_MINIMUM_LIMIT;
    }


    hoodMotor.configMotionCruiseVelocity(rpmToTicksPer100ms(ShooterConstants.HOOD_MOTOR_VELOCITY));
    hoodMotor.configMotionAcceleration(rpmToTicksPer100ms(ShooterConstants.HOOD_MOTOR_ACCELERATION));
    hoodMotor.configMotionSCurveStrength(ShooterConstants.HOOD_MOTOR_MOTION_SMOOTHING);

    hoodMotor.set(ControlMode.MotionMagic, hoodDegreesToTicks(degrees));
  }

  /**
   * Move the hood the specified amount of degrees based off of current location
   * @param degrees
   */
  public void relativeHoodAngle(double degrees) {
    setHoodAngle(hoodTicksToDegrees(hoodMotor.getClosedLoopTarget()) + degrees);
  }

  public boolean hoodMotionCommplete() {
    return (Math.abs(hoodMotor.getSelectedSensorPosition()-hoodMotor.getClosedLoopTarget())<=ShooterConstants.HOOD_TOLERANCE);
  }

  /**
   * Resets the hood encoder to set degrees
   * @param degrees angle for the hood encoder to be set to (in degrees)
   */
  public void resetHood(double degrees){
    hoodMotor.setSelectedSensorPosition(hoodDegreesToTicks(degrees));
  }

  /**
   * Stops the flywheel
   */
  public void stopFlywheel() {
      shooterMotor.set(0);
      isRunning = false;
  }

  public double getFlywheelVelocity() {
    return convertCPDToRPM(shooterMotor.getSelectedSensorVelocity());
  }

  public boolean getHoodLimit() {
    return hoodMotor.isFwdLimitSwitchClosed() == 1;
  }

}
