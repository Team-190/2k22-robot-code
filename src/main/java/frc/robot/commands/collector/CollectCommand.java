// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorSubsystem;

public class CollectCommand extends CommandBase {
    CollectorSubsystem collectorSubsystem;
    Timer timer;
    boolean tripped;

    public CollectCommand(RobotContainer robotContainer) {
        // Use addRequirements() here to declare subsystem dependencies.
        collectorSubsystem = robotContainer.collectorSubsystem;
        addRequirements(collectorSubsystem);
        timer = new Timer();
        tripped = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        collectorSubsystem.deploy();
        timer.reset();
        tripped = false;
    }

    @Override
    public void execute() {
        if (collectorSubsystem.getCurrent() >= 40) {
            timer.reset();
            timer.start();
            collectorSubsystem.stop();
            tripped = true;
        } else {
            if (tripped && !timer.hasElapsed(0.1)) {
                collectorSubsystem.stop();
            } else if (tripped && timer.hasElapsed(0.1)) {
                tripped = false;
                timer.stop();
                timer.reset();
            } else {
                collectorSubsystem.intake();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stop();
    }
}