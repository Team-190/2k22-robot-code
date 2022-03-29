// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleCollectCommand extends InstantCommand {

  CollectorSubsystem collectorSubsystem;
  double speed = 0;
  public ToggleCollectCommand(RobotContainer robotContainer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    collectorSubsystem = robotContainer.collectorSubsystem;
    this.speed = speed;
    addRequirements(collectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectorSubsystem.toggleCollector(speed);
  }
}
