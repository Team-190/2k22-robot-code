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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootGoodCommand extends SequentialCommandGroup {
  ShooterSubsystem shooterSubsystem;
  CollectorSubsystem collectorSubsystem;
  double speed = 0;
  /** Creates a new ShootGoodCommand. */
  public ShootGoodCommand(RobotContainer robotContainer, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    shooterSubsystem = robotContainer.shooterSubsystem;
    collectorSubsystem = robotContainer.collectorSubsystem;
    this.speed = speed;

    addCommands(
      new InstantCommand(()-> collectorSubsystem.upperBallPath(-.3)).withTimeout(.5),
      new RunCommand(()-> shooterSubsystem.flywheelManual(.55), shooterSubsystem)
    );
  }
}
