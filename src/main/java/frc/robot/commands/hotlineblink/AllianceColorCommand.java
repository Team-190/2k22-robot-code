package frc.robot.commands.hotlineblink;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HotlineBlinkSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem.Hat;

/** Sets LEDs to the color of the robot's alliance */
public class AllianceColorCommand extends CommandBase {
    private HotlineBlinkSubsystem hotlineBlinkSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Alliance alliance;

    /** Requires the blinkin */
    public AllianceColorCommand(RobotContainer robotContainer) {
        this.hotlineBlinkSubsystem = robotContainer.hotlineBlinkSubsystem;
        addRequirements(hotlineBlinkSubsystem);
    }

    /** Sets the color to the alliance color */
    @Override
    public void initialize() {
        setColor();
    }

    /** Changes the color if the alliance has changed */
    @Override
    public void execute() {
        Alliance newAlliance = DriverStation.getAlliance();
        if (newAlliance != alliance) {
            alliance = newAlliance;
        }
        setColor();
    }

    /** Sets the color depending on the alliance */
    private void setColor() {
        if (alliance == Alliance.Blue) {
            hotlineBlinkSubsystem.changeHat(Hat.BPMOcean);
        } else if (alliance == Alliance.Red) {
            hotlineBlinkSubsystem.changeHat(Hat.LightChaseRed);
        } else {
            hotlineBlinkSubsystem.changeHat(Hat.RainbowForest);
        }
    }
}