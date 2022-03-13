// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretManualCommand extends CommandBase {

  TurretSubsystem turretSubsystem = null;
  double setpoint;

  /**
   * Creates a new MoveTurretManualCommand.
   */
  public MoveTurretManualCommand(RobotContainer robotContainer, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = robotContainer.turretSubsystem;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.relativeTurretPID(this.setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}