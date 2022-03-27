package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends CommandBase {

    ShooterSubsystem shooterSubsystem = null;
    Double bottomRPM = null;
    Double topRPM = null;

    public RunShooterCommand(ShooterSubsystem shooterSubsystem, double bottomRPM, double topRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.bottomRPM = bottomRPM;
        this.topRPM = topRPM;

        // addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.flywheelPID(bottomRPM);
    }

    /**
     * At the end stops the shooter
     *
     * @param interrupted whether or not the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
    }
}
