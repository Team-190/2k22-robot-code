// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hotlineblink;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.SpitBallsOutCommand;
import frc.robot.subsystems.HotlineBlinkSubsystem.Hat;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitBallsWithColorCommand extends ParallelRaceGroup {
  /** Creates a new SpitBallsWithColor. */
  public SpitBallsWithColorCommand(RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(()-> robotContainer.hotlineBlinkSubsystem.changeHat(Hat.HotPink)),
      new SpitBallsOutCommand(robotContainer)

    );
  }
}
