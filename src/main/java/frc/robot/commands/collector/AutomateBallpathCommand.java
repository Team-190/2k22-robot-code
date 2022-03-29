// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;

public class AutomateBallpathCommand extends CommandBase {
  CollectorSubsystem collectorSubsystem;
  /** Creates a new AutomateBallpath. */
  public AutomateBallpathCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    collectorSubsystem = robotContainer.collectorSubsystem;
    addRequirements(collectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectorSubsystem.upperBallPath(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collectorSubsystem.detectBallpath()) {
      collectorSubsystem.upperBallPath(0); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorSubsystem.upperBallPath(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collectorSubsystem.detectBallpath();
  }
}
