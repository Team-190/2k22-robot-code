package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class RampBottomShooterCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    double rpm;

    public RampBottomShooterCommand(RobotContainer robotContainer, double rpm) {
        this.shooterSubsystem = robotContainer.shooterSubsystem;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.flywheelPID(rpm);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.flywheelAtTargetRPM();
    }
}
