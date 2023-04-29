// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SearchTargetCommand extends CommandBase {

  RobotContainer robotContainer = null;
  TurretSubsystem turretSubsystem = null;
  LimeLightSubsystem limeLightSubsystem = null;
  int directionOverride = 1;

  /** Creates a new SearchTargetCommand. */
  public SearchTargetCommand(RobotContainer robotContainer, boolean directionOverride) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.robotContainer = robotContainer;
    this.turretSubsystem = robotContainer.turretSubsystem;
    this.directionOverride = (directionOverride ? -1 : 1);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (turretSubsystem.isMotionComplete() && !limeLightSubsystem.targetFound()) {
        turretSubsystem.relativeTurretPID(turretSubsystem.degreesToTicks(turretSubsystem.getDirection() * directionOverride * TurretConstants.TURRET_STEP_SIZE));
    } else if (limeLightSubsystem.targetFound()) {
      new TargetLeadLagCommand(robotContainer);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
