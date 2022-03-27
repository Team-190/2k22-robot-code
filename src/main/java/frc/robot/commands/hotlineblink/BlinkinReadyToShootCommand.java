// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hotlineblink;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HotlineBlinkSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem.Hat;

public class BlinkinReadyToShootCommand extends CommandBase {
  HotlineBlinkSubsystem hotlineBlinkSubsystem;
  TurretSubsystem turretSubsystem;
  ShooterSubsystem shooterSubsystem;
  LimeLightSubsystem limeLightSubsystem;
  RobotContainer robotContainer;

  /** Creates a new BlinkinReadyToShootCommand. */
  public BlinkinReadyToShootCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hotlineBlinkSubsystem = robotContainer.hotlineBlinkSubsystem;
    this.turretSubsystem = robotContainer.turretSubsystem;
    this.shooterSubsystem = robotContainer.shooterSubsystem;
    this.limeLightSubsystem = robotContainer.limeLightSubsystem;
    this.robotContainer = robotContainer;
    addRequirements(hotlineBlinkSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limeLightSubsystem.getVision() && shooterSubsystem.getIsRunning()) {
      hotlineBlinkSubsystem.changeHat(Hat.Gold);
    } 
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
