package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {

    ShooterSubsystem shooterSubsystem;
    double bottomRPM;
    double topRPM;

    public ShootCommand(RobotContainer robotContainer) {
        this.shooterSubsystem = robotContainer.shooterSubsystem;
        this.bottomRPM = robotContainer.bottomRPMChooser.getSelected();
        this.topRPM = robotContainer.topRPMChooser.getSelected();

        SmartDashboard.putNumber("Target Bottom RPM", bottomRPM);
        SmartDashboard.putNumber("Target Top RPM", topRPM);


        shooterSubsystem.topShooterMotor.config_kF(0, (1023 * 0.15 / shooterSubsystem.rpmToTicksPer100ms(topRPM)));
        shooterSubsystem.bottomShooterMotor.config_kF(0, (1023 * 0.15 / shooterSubsystem.rpmToTicksPer100ms(bottomRPM)));

        addCommands(
                new ParallelCommandGroup(
                    new RampBottomShooterCommand(robotContainer, bottomRPM),
                    new RampTopShooterCommand(robotContainer, topRPM)),
                new RunShooterCommand(shooterSubsystem, bottomRPM, topRPM));
    }

    /**
     * At the end stops the shooter
     *
     * @param interrupted whether or not the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }


}
