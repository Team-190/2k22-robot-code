// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class hoodToAngleCommand extends CommandBase {
  ShooterSubsystem shooterSubsystem;
  double angle = ShooterConstants.HOOD_MINIMUM_LIMIT;

  /** Creates a new hoodToAngleCommand. */
  public hoodToAngleCommand(RobotContainer robotContainer, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = robotContainer.shooterSubsystem;
    this.angle = angle;
    // addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setHoodAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.hoodMotionCommplete();
  }
}
