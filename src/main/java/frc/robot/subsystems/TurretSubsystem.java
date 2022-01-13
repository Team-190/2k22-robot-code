// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class TurretSubsystem extends PIDSubsystem {

  WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.TurretConstants.TURRET_CHANNEL);



  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(double P, double I, double D) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
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

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
