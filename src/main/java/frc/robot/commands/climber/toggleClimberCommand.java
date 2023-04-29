// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class toggleClimberCommand extends CommandBase {
  ClimberSubsystem climberSubsystem;
  TurretSubsystem turretSubsystem;
  LimeLightSubsystem limeLightSubsystem;
  boolean toggle = false;

  /** Creates a new toggleClimberCommand. */
  public toggleClimberCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    climberSubsystem = robotContainer.climberSubsystem;
    turretSubsystem = robotContainer.turretSubsystem;
    limeLightSubsystem = robotContainer.limeLightSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLightSubsystem.setVision(false);
    turretSubsystem.turretPID(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.togglePivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretSubsystem.isMotionComplete();
  }
}
