// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HotlineBlinkSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem.Hat;

public class LowPortCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  HotlineBlinkSubsystem hotlineBlinkSubsystem;
  /** Creates a new LowPortCommand. */
  public LowPortCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.

    shooterSubsystem = robotContainer.shooterSubsystem;
    hotlineBlinkSubsystem = robotContainer.hotlineBlinkSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooterSubsystem.flywheelPID(1500);
      shooterSubsystem.setHoodAngle(27);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getIsRunning()) {
      hotlineBlinkSubsystem.changeHat(Hat.Black);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooterSubsystem.getToggle();
  }
}
