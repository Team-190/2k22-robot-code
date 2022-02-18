// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class ShootWhileMovingCommand extends CommandBase {
  private TurretSubsystem turretSubsystem = null;
  private LimeLightSubsystem limeLightSubsystem = null;
  private RobotContainer robotContainer = null;
  private double lastSeen = 1;

  /** Creates a new VisionCommand. */
  public ShootWhileMovingCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = robotContainer.turretSubsystem;
    this.limeLightSubsystem = robotContainer.limeLightSubsystem;
    this.robotContainer = robotContainer;
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limeLightSubsystem.targetFound()) {
      lastSeen = limeLightSubsystem.degreesAskew();
    } 
    if (turretSubsystem.turretUnwrap());
    // Checks to see if the motion is complete and if there is no target then scans until target found.
    else if (turretSubsystem.isMotionComplete() && !limeLightSubsystem.targetFound()) {
      turretSubsystem.relativeTurretPID(turretSubsystem.degreesToTicks(lastSeen));
    } else new TargetLeadLagCommand(robotContainer);
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
