package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class RampTopShooterCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    double rpm;

    public RampTopShooterCommand(RobotContainer robotContainer, double rpm) {
        this.shooterSubsystem = robotContainer.shooterSubsystem;
        this.rpm = rpm;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.topShooterPID(rpm);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.topAtTargetRPM(rpm);
    }
}
