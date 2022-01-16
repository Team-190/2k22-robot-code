// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimeLightSubsystem;

public class TurretSubsystem extends PIDSubsystem {

  WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.TurretConstants.TURRET_CHANNEL);
  ShuffleboardTab tab = Shuffleboard.getTab("Turret");



  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(double P, double I, double D, double F) {
      super(
          // The PIDController used by the subsystem
          new PIDController(P, I, D));
          
          configPIDF(
                  turretMotor,
                  TurretConstants.P,
                  TurretConstants.I,
                  TurretConstants.D,
                  TurretConstants.F);
          turretMotor.configAllowableClosedloopError(0, 0.5);
          turretMotor.configClosedLoopPeakOutput(0, 0.15);
          turretMotor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, TurretConstants.PID_LOOPTYPE, TurretConstants.TIMEOUT_MS);
          turretMotor.setInverted(true);
          turretMotor.setSensorPhase(true);

      
      tab.addNumber("Turret Encoder Position", () -> turretMotor.getSelectedSensorPosition());
      tab.addNumber("Turret Encoder Error", () -> turretMotor.getClosedLoopError());
  }

  /**
    * Configures Turret PIDF
    * @param motorController The motor controller to configure
    * @param P proportional value
    * @param I integral value
    * @param D derivative value
    * @param F feed forward value
    */
    public void configPIDF(WPI_TalonSRX motorController, double P, double I, double D, double F) {
      motorController.config_kP(TurretConstants.SLOT_ID, P);
      motorController.config_kI(TurretConstants.SLOT_ID, I);
      motorController.config_kD(TurretConstants.SLOT_ID, D);
      //motorController.config_kF(TurretConstants.SLOT_ID, F);
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
   * Move turret towards vision target
   */
  public void turretVision(){
    relativeTurretPID(degreesToTicks(LimeLightSubsystem.degreesAskew()));
  }

  /**
   * Convert from ticks to degrees
   * @param ticks current encoder tick value
   * @return current degree amount
   */
  public double ticksToDegrees (int ticks) {
    return ticks * 3.6;
  }

  /**
   * Convert from degrees to ticks
   * @param degrees current degree amount
   * @return current encoder tick value
   */
  public int degreesToTicks (double degrees){
    double ticks = degrees/3.6;
    return (int)Math.round(ticks);
  }

  /**
   * Move turret to setpoint
   * @param setpoint encoder tick value for turret to move to
   */
  public void turretPID(double setpoint) {
    turretMotor.set(ControlMode.Position, setpoint);
  }


  /**
   * Move turret relatively by setpoint
   * @param setpoint encoder tick value to move turret by
   */
  public void relativeTurretPID(double setpoint) {
    turretPID(turretMotor.getClosedLoopError() + turretMotor.getSelectedSensorPosition() + setpoint);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }


  /**
   * Reset the encoder position
   */
  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    

  }
}
