// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem.Hat;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitBallsOutCommand extends SequentialCommandGroup {

  ShooterSubsystem shooterSubsystem;
  CollectorSubsystem collectorSubsystem;

  /** Creates a new SpitBallsOutCommand. */
  public SpitBallsOutCommand(RobotContainer robotContainer) {

    shooterSubsystem = robotContainer.shooterSubsystem;
    collectorSubsystem = robotContainer.collectorSubsystem;
    addRequirements(shooterSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(()-> shooterSubsystem.flywheelPID(1000))
      .alongWith(new InstantCommand(()-> shooterSubsystem.setHoodAngle(27))).withTimeout(.5),
      new RunCommand(()-> collectorSubsystem.upperBallPath(0.7)).withTimeout(2),
      new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
      new InstantCommand(()-> collectorSubsystem.upperBallPath(0))

    );
  }
}
