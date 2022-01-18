package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ScanCommand extends CommandBase {
    LimeLightSubsystem limeLightSubsystem;
    TurretSubsystem turretSubsystem;
    public ScanCommand(RobotContainer robotContainer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.limeLightSubsystem = robotContainer.limeLightSubsystem;
        this.turretSubsystem = robotContainer.turretSubsystem;
//        addRequirements(turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        turretSubsystem.scanForTarget();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return limeLightSubsystem.targetFound();
    }
}
