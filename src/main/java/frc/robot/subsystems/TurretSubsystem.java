// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends PIDSubsystem {

  WPI_TalonFX turretMotor = new WPI_TalonFX(Constants.TurretConstants.TURRET_CHANNEL);
  DigitalInput turretLimit = new DigitalInput(TurretConstants.TURRET_LIMIT_CHANNEL);
  ShuffleboardTab tab = Shuffleboard.getTab("Turret");

  LimeLightSubsystem limeLightSubsystem = null;

  double lastSeen = 0;
  double TURRET_MAXIMUM_LIMIT = degreesToTicks(186); // TODO: Find this
  double TURRET_MINIMUM_LIMIT = degreesToTicks(-186); // TODO: Find this
  int turnToDirection = 1;
  int defaultDirection = 1;




  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(double P, double I, double D, LimeLightSubsystem limeLightSubsystem) {
      super(
          // The PIDController used by the subsystem
          new PIDController(P, I, D));

      this.limeLightSubsystem = limeLightSubsystem;
      // turretMotor.setStatusFramePeriod(frame, periodMs)
      turretMotor.configFactoryDefault();

      configPIDF(
              turretMotor,
              TurretConstants.P,
              TurretConstants.I,
              TurretConstants.D,
              1023 / rpmToTicksPer100ms(TurretConstants.TURRET_MAX_RPM));

      turretMotor.configAllowableClosedloopError(0, TurretConstants.TOLERANCE);
      turretMotor.configClosedLoopPeakOutput(0, 1);
      turretMotor.configClosedLoopPeriod(0, 1);
      turretMotor.configClosedloopRamp(0.00);
      turretMotor.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor, TurretConstants.PID_LOOPTYPE, TurretConstants.TIMEOUT_MS);
      turretMotor.setInverted(false);

      turretMotor.setNeutralMode(NeutralMode.Brake);

      turretMotor.configForwardSoftLimitEnable(true);
      turretMotor.configReverseSoftLimitEnable(true);
      turretMotor.configForwardSoftLimitThreshold(degreesToTicks(190));
      turretMotor.configReverseSoftLimitThreshold(degreesToTicks(-190));

      
      
      tab.addNumber("LimelightAngleTicks", () -> degreesToTicks(limeLightSubsystem.degreesAskew()));
      tab.addNumber("LimelightAngleDegrees", () -> limeLightSubsystem.degreesAskew());
      
  }

  /**
    * Configures Turret PIDF
    * @param motorController The motor controller to configure
    * @param P proportional value
    * @param I integral value
    * @param D derivative value
    * @param F feedforward value
    */
    public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
      motorController.config_kP(TurretConstants.SLOT_ID, P);
      motorController.config_kI(TurretConstants.SLOT_ID, I);
      motorController.config_kD(TurretConstants.SLOT_ID, D);
      motorController.config_kF(TurretConstants.SLOT_ID, F);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  /**
   * Set turret motor speed 
   * @param speed set motor output [-1,1]
   */
  public void turretManual(double speed) {
    turretMotor.set(speed);
  }

  /**
   * Unwraps the turret if it gets past it's maximum or minimum limit
   * @return True if the turret gets past the upper or lower limit else False
   */
  public boolean turretUnwrap() {
    if (getTurretPosition() >= TURRET_MAXIMUM_LIMIT) {
      relativeTurretPID(degreesToTicks(-360));
      return true;
    } else if (getTurretPosition() <= TURRET_MINIMUM_LIMIT) {
      relativeTurretPID(degreesToTicks(360));
      return true;
    }
    return false;
  }
  /**
   * Move turret towards vision target
   */
  public void turretVision(){
    double degrees = limeLightSubsystem.degreesAskew();

    // Takes in degrees off from target, converts it into ticks, move turret by those ticks
    if (limeLightSubsystem.getVision() && limeLightSubsystem.targetFound() && !visionWithinTolerance()) {
      turretPID(getTurretPosition() + degreesToTicks(degrees));
    }
  }

  /**
   * Finds if vision is within tolerance
   */
  public boolean visionWithinTolerance() {
    return Math.abs(ticksToDegrees(getTurretPosition()) - limeLightSubsystem.degreesAskew()) < ticksToDegrees(TurretConstants.TOLERANCE);
  }

  /**
   * Checks if the PID motion is complete
   */
  public boolean isMotionComplete(){
    return (Math.abs(getTurretPosition()-turretMotor.getClosedLoopTarget())<=TurretConstants.TOLERANCE);
  }

  /**
   * Convert from ticks to degrees
   * @param ticks current encoder tick value
   * @return current degree amount
   */
  public double ticksToDegrees (double ticks) {
    return ticks / TurretConstants.TICKS_PER_DEGREE;
  }

  /**
   * Convert from degrees to ticks
   * @param degrees current degree amount
   * @return current encoder tick value
   */
  public double degreesToTicks (double degrees){
    return degrees * TurretConstants.TICKS_PER_DEGREE;
  }

  /**
   * Get the position of the Turret (Positive is counter Clockwise, 0 is front of robot)
   * @return the position of the turret in ticks
   */
  public double getTurretPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  /**
   * Get the angle of the Turret (Positive is counter Clockwise, 0 is front of robot)
   * @return the angle of the turret in degrees
   */
  public double getTurretAngle() {
    return ticksToDegrees(getTurretPosition());
  }

  /**
   * Move turret to setpoint
   * @param setpoint encoder tick value for turret to move to
   */
  public void turretPID(double setpoint) {
    
    if (setpoint > TURRET_MAXIMUM_LIMIT) {
      setpoint -= degreesToTicks(360);
    } else if (setpoint < TURRET_MINIMUM_LIMIT) {
      setpoint += degreesToTicks(360);
    }

    turretMotor.configMotionCruiseVelocity(rpmToTicksPer100ms(TurretConstants.TURRET_MOTOR_VELOCITY));
    turretMotor.configMotionAcceleration(rpmToTicksPer100ms(TurretConstants.TURRET_MOTOR_ACCELERATION));
    turretMotor.configMotionSCurveStrength(TurretConstants.TURRET_MOTOR_MOTION_SMOOTHING);

    turretMotor.set(ControlMode.MotionMagic, setpoint);
  }


  /**
   * Move turret relatively by setpoint 
   * @param setpoint encoder tick value to move turret by
   */
  public void relativeTurretPID(double setpoint) {
    turretPID(turretMotor.getClosedLoopTarget() + setpoint);
  }

  /**
   * Converts rotations per minute to ticks per 100ms
   * Multiplies rotations per minute by ticks per rotation
   * Divides by 600 to convert from minute to millisecond
   * @param rpm input of rotations per minute
   * @return output of ticks per 100ms
   */
  public double rpmToTicksPer100ms(double rpm) {
    return rpm * TurretConstants.TICKS_PER_ROTATION / 600;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return turretMotor.getSelectedSensorPosition();
  }

  /**
   * Sets the direction for the turret to search in (default direction)
   */
  public void setDirection() {
    setDirection(this.defaultDirection);
  }

  /**
   * Sets the direction for the turret to search in
   * @param direction the direction input for the turret to search to [-1 or 1]
   */
  public void setDirection(int direction) {
    this.turnToDirection = direction;
  }

  /**
   * Gets the direction of the turret to search in
   * @return the direction for the robot to search in [-1 or 1]
   */
  public int getDirection() {
    return this.turnToDirection;
  }

  /**
   * Gets the state of the turret limit switch
   * @return true if the turret is over the magnet else false
   */
  public boolean getTurretLimit() {
    return turretLimit.get();
  }


  /**
   * Reset the encoder position
   */
  public void resetEncoder(double ticks) {
    turretMotor.setSelectedSensorPosition(ticks);
  }

  

  /**
   * Checks to see if the turret is within climber tolerance
   * @return True if the turret is within tolerence else false
   */
  public boolean getPivotTolerance() {
    double currentPosition = getTurretPosition();
    boolean ethernetSwitchTolerance = currentPosition < -3000 && currentPosition > -30000;
    boolean turretBehindTolerance = currentPosition < -50000 || currentPosition > 60000;
    return ethernetSwitchTolerance || turretBehindTolerance;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Position", getTurretPosition());
    SmartDashboard.putNumber("Turret Encoder Error", turretMotor.getClosedLoopError());
    SmartDashboard.putNumber("Turret PID Target", turretMotor.getClosedLoopTarget());
    // SmartDashboard.putBoolean("Turret Limit", getTurretLimit());

    if (getTurretLimit()) {
      // turretMotor.configClearPositionOnLimitR(true, 20);
    }

  }
}
