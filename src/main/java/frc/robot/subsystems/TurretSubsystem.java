// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

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
          turretMotor.configAllowableClosedloopError(0, 100);
          turretMotor.configClosedLoopPeakOutput(0, 1.0);

      
      tab.addNumber("Turret Encoder Position", () -> turretMotor.getSelectedSensorPosition());
      tab.addNumber("Turret Encoder Setpoint", () -> turretMotor.getClosedLoopTarget());
      tab.addNumber("Turret Encoder Error", () -> turretMotor.getClosedLoopError());
  }

  /**
    * Configures Turret PIDF
    *
    * @param motorController The motor controller to configure
    * @param P proportional value
    * @param I integral value
    * @param D derivative value
    * @param F feed forward value
    */
    public void configPIDF(WPI_TalonSRX motorController, double P, double I, double D, double F) {
      motorController.config_kP(TurretConstants.SLOT_ID, P);
      motorController.config_kP(TurretConstants.SLOT_ID, I);
      motorController.config_kP(TurretConstants.SLOT_ID, D);
      motorController.config_kP(TurretConstants.SLOT_ID, F);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  /**
   * set turret speed 
   * @param speed set motor output [-1,1]
   */
  public void turretManual(double speed) {
    turretMotor.set(speed);
  }


  public void turretPID(double setpoint) {
    turretMotor.set(ControlMode.MotionMagic, setpoint);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  @Override
  public void periodic() {
    

  }
}
