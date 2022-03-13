// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends CommandBase {
  ClimberSubsystem climberSubsystem = null;
  
  /** Creates a new ClimberExtendCommand. */
  public ClimberExtendCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.

    climberSubsystem = robotContainer.climberSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.extendClimber(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.extendClimber(0);
    climberSubsystem.brakeActuate(false);
    climberSubsystem.setStage(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.getClimberPosition() >= 51000;
  }
}
