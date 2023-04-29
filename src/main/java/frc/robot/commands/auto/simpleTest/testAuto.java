// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.simpleTest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.TrajectoryFollowerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testAuto extends SequentialCommandGroup {
  /** Creates a new testAuto. */
  public testAuto(RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    robotContainer.drivetrainSubsystem.resetAll();
    robotContainer.drivetrainSubsystem.resetGyro(true);
    addCommands(
      
      new TrajectoryFollowerCommand(robotContainer, testTrajectory.START)
    
    );
    // new TestVoltCommand(robotContainer).withTimeout(2));
    // new RunCommand(()-> robotContainer.drivetrainSubsystem.westCoastDrive(1,1,false), robotContainer.drivetrainSubsystem));
  }
}
