// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSetpointCommand extends CommandBase {
  
  TurretSubsystem turretSubsystem = null;
  LimeLightSubsystem limeLightSubsystem = null;
  double setpoint;

  /**
   * Creates a new TurretSetpointCommand.
   */
  public TurretSetpointCommand(RobotContainer robotContainer, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = robotContainer.turretSubsystem;
    this.setpoint = setpoint;
    this.limeLightSubsystem = robotContainer.limeLightSubsystem;

    addRequirements(this.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.turretPID(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pass, no need to constantly update a detected variable when it automatically updates anyway
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (limeLightSubsystem.getVision() && limeLightSubsystem.targetFound()) turretSubsystem.turretVision();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretSubsystem.isMotionComplete() || (limeLightSubsystem.getVision() ? limeLightSubsystem.targetFound() : false);
  }
}