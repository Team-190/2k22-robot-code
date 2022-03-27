// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.threeBallAutoTurn;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Turret.TurretSetpointCommand;
import frc.robot.commands.Turret.VisionCommand;
import frc.robot.commands.auto.TrajectoryFollowerCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.ShootDistanceCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class threeBallAutoTurn extends SequentialCommandGroup {
  /** Creates a new twoBallAuto. */
  public threeBallAutoTurn(RobotContainer robotContainer) {

    robotContainer.turretSubsystem.resetEncoder(robotContainer.turretSubsystem.degreesToTicks(-180));
    robotContainer.limeLightSubsystem.setVision(true);
    robotContainer.drivetrainSubsystem.resetAll();
    robotContainer.drivetrainSubsystem.resetGyro(true);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new VisionCommand(robotContainer).withTimeout(0.1),
      new ShootDistanceCommand(robotContainer).withTimeout(2),
      new TrajectoryFollowerCommand(robotContainer, threeBallTurnTrajectory.START)
        .alongWith(new InstantCommand(()-> robotContainer.collectorSubsystem.toggleCollector(.75))),
      new ShootDistanceCommand(robotContainer).withTimeout(2),
      new RunCommand(()-> robotContainer.collectorSubsystem.upperBallPath(0.7)).withTimeout(2),
      new InstantCommand(()-> robotContainer.shooterSubsystem.setToggle(!robotContainer.shooterSubsystem.getToggle())),
      new InstantCommand(()-> robotContainer.collectorSubsystem.upperBallPath(0)),
      new TrajectoryFollowerCommand(robotContainer, threeBallTurnTrajectory.PLAYERSTATION),
      new RunCommand(()-> robotContainer.collectorSubsystem.upperBallPath(0.7)).withTimeout(2),
      new InstantCommand(()-> robotContainer.shooterSubsystem.setToggle(!robotContainer.shooterSubsystem.getToggle())),
      new InstantCommand(()-> robotContainer.collectorSubsystem.upperBallPath(0)),
      new InstantCommand(()-> robotContainer.collectorSubsystem.toggleCollector(0))


      /*
      new VisionCommand(robotContainer),
      new ShootDistanceCommand(robotContainer, 120),
      new TrajectoryFollowerCommand(robotContainer, twoBallTrajectory.START)
        .alongWith(new InstantCommand(()-> robotContainer.collectorSubsystem.toggleCollector(.75))),
      new AutoShootCommand(robotContainer, 3000).withTimeout(2),
      new InstantCommand(()-> robotContainer.collectorSubsystem.toggleCollector(0)),
      new InstantCommand(()-> robotContainer.shooterSubsystem.flywheelManual(0))
      */
    );
  }
}
