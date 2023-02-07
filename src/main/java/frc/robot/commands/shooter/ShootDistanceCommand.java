// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HotlineBlinkSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem.Hat;

public class ShootDistanceCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  LimeLightSubsystem limeLightSubsystem;
  HotlineBlinkSubsystem hotlineBlinkSubsystem;
  double distance;
  double minDistance = 200;
  boolean toggle = false;

  /** Creates a new ShootDistanceCommand. */
  public ShootDistanceCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooterSubsystem = robotContainer.shooterSubsystem;
    this.limeLightSubsystem = robotContainer.limeLightSubsystem;
    this.hotlineBlinkSubsystem = robotContainer.hotlineBlinkSubsystem;
    addRequirements(shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.distance = limeLightSubsystem.getDistanceToTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooterSubsystem.getToggle()) {
      shooterSubsystem.stopFlywheel();

    } else {
      hotlineBlinkSubsystem.changeHat(Hat.Gold);
    }
    if (limeLightSubsystem.getVision()) {
      if (limeLightSubsystem.targetFound()){
        this.distance = limeLightSubsystem.getDistanceToTarget();
      }
      shooterSubsystem.adjustHood(distance);
      if (shooterSubsystem.getToggle()) {
        shooterSubsystem.adjustFlywheel(distance);
      }
    } else {
      shooterSubsystem.adjustHood(minDistance);
      if (shooterSubsystem.getToggle()) {
        shooterSubsystem.adjustFlywheel(minDistance);
      }
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
    return false;
  }
}
