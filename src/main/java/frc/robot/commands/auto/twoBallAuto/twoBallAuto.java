// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.twoBallAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Turret.TurretSetpointCommand;
import frc.robot.commands.auto.TrajectoryFollowerCommand;
import frc.robot.commands.shooter.AutoShootCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class twoBallAuto extends SequentialCommandGroup {
  /** Creates a new twoBallAuto. */
  public twoBallAuto(RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TrajectoryFollowerCommand(robotContainer, twoBallTrajectory.START)
        .alongWith(new InstantCommand(()-> robotContainer.collectorSubsystem.toggleCollector(.75))),
      new InstantCommand(()-> robotContainer.shooterSubsystem.flywheelPID(5000)),
      new TrajectoryFollowerCommand(robotContainer, twoBallTrajectory.TURN)
        .alongWith(new TurretSetpointCommand(robotContainer, -80000).andThen(new AutoShootCommand(robotContainer, 5000)))
      

      //new collect command
      //new shoot command
    );
  }
}
