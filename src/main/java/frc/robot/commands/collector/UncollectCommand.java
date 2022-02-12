// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;

public class UncollectCommand extends CommandBase {

    CollectorSubsystem collectorSubsystem;
    private boolean done = false;

    public UncollectCommand(RobotContainer robotContainer) {
        collectorSubsystem = robotContainer.collectorSubsystem;
        addRequirements(collectorSubsystem);
    }

    // Pull collector back in
    @Override
    public void execute() {
        collectorSubsystem.undeploy();
        collectorSubsystem.stop();
    }
}