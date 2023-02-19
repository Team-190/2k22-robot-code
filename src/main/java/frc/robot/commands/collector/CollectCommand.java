// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;

public class CollectCommand extends CommandBase {
  CollectorSubsystem collectorSubsystem;
  double speed = 0;
  /** Creates a new CollectCommand. */
  public CollectCommand(RobotContainer robotContainer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putString("CollectCommand", "Created");
    this.speed = speed;
    collectorSubsystem = robotContainer.collectorSubsystem;
    addRequirements(collectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("CollectCommand", "Scheduled");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    collectorSubsystem.collect(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorSubsystem.collect(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
