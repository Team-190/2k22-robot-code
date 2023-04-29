// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem = null;
  CollectorSubsystem collectorSubsystem = null;
  double rpm = 0;

  /** Creates a new ShootCommand. */
  public AutoShootCommand(RobotContainer robotContainer, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooterSubsystem = robotContainer.shooterSubsystem;
    this.collectorSubsystem = robotContainer.collectorSubsystem;
    this.rpm = rpm;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.flywheelPID(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.flywheelAtTargetRPM()) {
      collectorSubsystem.upperBallPath(.7);
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
