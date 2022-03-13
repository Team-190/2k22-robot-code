// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends PIDSubsystem {

  WPI_TalonFX turretMotor = new WPI_TalonFX(Constants.TurretConstants.TURRET_CHANNEL);
  ShuffleboardTab tab = Shuffleboard.getTab("Turret");

  LimeLightSubsystem limeLightSubsystem = null;

  double lastSeen = 0;
  double TURRET_MAXIMUM_LIMIT = 1000000; // TODO: Find this
  double TURRET_MINIMUM_LIMIT = -1000000; // TODO: Find this
  int turnToDirection = 1;
  int defaultDirection = 1;




  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(double P, double I, double D, LimeLightSubsystem limeLightSubsystem) {
      super(
          // The PIDController used by the subsystem
          new PIDController(P, I, D));

      this.limeLightSubsystem = limeLightSubsystem;
      

      configPID(
              turretMotor,
              TurretConstants.P,
              TurretConstants.I,
              TurretConstants.D);
      turretMotor.configAllowableClosedloopError(0, TurretConstants.TOLERANCE);
      turretMotor.configClosedLoopPeakOutput(0, .5);
      turretMotor.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor, TurretConstants.PID_LOOPTYPE, TurretConstants.TIMEOUT_MS);
      turretMotor.setInverted(false);

      
      
      tab.addNumber("LimelightAngleTicks", () -> degreesToTicks(limeLightSubsystem.degreesAskew()));
      tab.addNumber("LimelightAngleDegrees", () -> limeLightSubsystem.degreesAskew());
  }

  /**
    * Configures Turret PIDF
    * @param motorController The motor controller to configure
    * @param P proportional value
    * @param I integral value
    * @param D derivative value
    */
    public void configPID(WPI_TalonFX motorController, double P, double I, double D) {
      motorController.config_kP(TurretConstants.SLOT_ID, P);
      motorController.config_kI(TurretConstants.SLOT_ID, I);
      motorController.config_kD(TurretConstants.SLOT_ID, D);
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
    if (turretMotor.getSelectedSensorPosition() >= TURRET_MAXIMUM_LIMIT) {
      relativeTurretPID(degreesToTicks(-360));
      return true;
    } else if (turretMotor.getSelectedSensorPosition() <= TURRET_MINIMUM_LIMIT) {
      relativeTurretPID(degreesToTicks(360));
      return true;
    }
    return false;
  }
  
  /**
   * Move turret towards vision target
   */
  public void turretVision(){
    // Takes in degrees off from target, converts it into ticks, move turret by those ticks
    relativeTurretPID(degreesToTicks(limeLightSubsystem.degreesAskew()));
  }

  /**
   * Checks if the PID motion is complete
   */
  public boolean isMotionComplete(){
    return (Math.abs(turretMotor.getSelectedSensorPosition()-turretMotor.getClosedLoopTarget())<=TurretConstants.TOLERANCE); // TODO: set tolerance
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
   * Get the angle of the Turret (Positive is counter Clockwise, 0 is front of robot)
   * @return the angle of the turret in degrees
   */
  public double getTurretAngle() {
    return ticksToDegrees(turretMotor.getSelectedSensorPosition());
  }

  /**
   * Move turret to setpoint
   * @param setpoint encoder tick value for turret to move to
   */
  public void turretPID(double setpoint) {
    double offset = 0;
    if (turretMotor.getClosedLoopTarget() > TURRET_MAXIMUM_LIMIT) {
      offset = setpoint - TURRET_MAXIMUM_LIMIT;
      setpoint = TURRET_MINIMUM_LIMIT + offset;
    } else if (turretMotor.getClosedLoopTarget() > TURRET_MINIMUM_LIMIT) {
      offset = setpoint + TURRET_MINIMUM_LIMIT;
      setpoint = TURRET_MAXIMUM_LIMIT + offset;
    }

    if (false) {
      turretMotor.configMotionCruiseVelocity(rpmToTicksPer100ms(TurretConstants.TURRET_MOTOR_VELOCITY));
      turretMotor.configMotionAcceleration(rpmToTicksPer100ms(TurretConstants.TURRET_MOTOR_ACCELERATION));
      turretMotor.configMotionSCurveStrength(TurretConstants.TURRET_MOTOR_MOTION_SMOOTHING);
    }

    turretMotor.set(ControlMode.Position, setpoint);
  }


  /**
   * Move turret relatively by setpoint
   * @param setpoint encoder tick value to move turret by
   */
  public void relativeTurretPID(double setpoint) {
    // turretMotor.getClosedLoopTarget()
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
    this.turnToDirection = this.defaultDirection;
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
   * Reset the encoder position
   */
  public void resetEncoder(double ticks) {
    turretMotor.setSelectedSensorPosition(ticks);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Position", turretMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Turret Encoder Error", turretMotor.getClosedLoopError());

  }
}
