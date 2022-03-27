// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbExtendRightCommand extends SequentialCommandGroup {

  ClimberSubsystem climberSubsystem;
  /** Creates a new ClimbMidBarCommand. */
  public ClimbExtendRightCommand(RobotContainer robotContainer, double setpoint) {

    climberSubsystem = robotContainer.climberSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimberRightBrakeCommand(robotContainer, false),
      new ClimberSetpointCommand(robotContainer, setpoint),
      new ClimberRightBrakeCommand(robotContainer, true)

    );
  }
}
