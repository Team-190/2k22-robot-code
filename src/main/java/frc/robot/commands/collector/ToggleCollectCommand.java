package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ToggleCollectCommand extends InstantCommand {
    private final RobotContainer robotContainer;

    public ToggleCollectCommand(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    // Get the state of the collector, then schedule the opposite command
    @Override
    public void execute() {
        switch (robotContainer.collectorSubsystem.getState()) {
            case DEPLOYED:
                new UncollectCommand(robotContainer).schedule();
                break;
            case UNDEPLOYED:
                new CollectCommand(robotContainer).schedule();
                break;
        }
    }
}