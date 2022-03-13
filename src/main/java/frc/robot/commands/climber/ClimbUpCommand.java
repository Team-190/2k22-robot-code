// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbUpCommand extends SequentialCommandGroup {
  ClimberSubsystem climberSubsystem;
  /** Creates a new ClimbUpCommand. */
  public ClimbUpCommand(RobotContainer robotContainer) {
    climberSubsystem = robotContainer.climberSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> climberSubsystem.clamperOpen()),
      new InstantCommand(()-> climberSubsystem.brakeActuate(true)).withTimeout(1),
      new ClimberExtendCommand(robotContainer)
    );
  }
}
