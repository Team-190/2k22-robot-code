// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;

public class ToggleCollectCommand extends CommandBase {
  CollectorSubsystem collectorSubsystem = null;
  double speed = 0;
  boolean toggle = false;

  /** Creates a new ToggleCollectCommand. */
  public ToggleCollectCommand(RobotContainer robotContainer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collectorSubsystem = robotContainer.collectorSubsystem;
    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toggle = !toggle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggle = !toggle;
    if (toggle) {
      collectorSubsystem.collect(speed);
      collectorSubsystem.extend();

    } else {
      collectorSubsystem.collect(0);
      collectorSubsystem.retract();
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
