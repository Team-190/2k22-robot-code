package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.input.AttackThree;
import frc.robot.input.AttackThree.AttackThreeAxis;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Default Drive Command */
public class DefaultTankDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final AttackThree leftStick;
    private final AttackThree rightStick;

    public DefaultTankDriveCommand(RobotContainer robotContainer) {
        this.leftStick = robotContainer.leftStick;
        this.rightStick = robotContainer.rightStick;

        this.drivetrainSubsystem = robotContainer.drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {}

    /**
    * Take the values from the controllers and the current styles in the robot container and set the
    * drive based upon it
    */
    @Override
    public void execute() {

        double throttleLeftValue = 0.0;
        double throttleRightValue = 0.0;

        throttleLeftValue = leftStick.getAxis(AttackThreeAxis.Y);
        throttleRightValue = rightStick.getAxis(AttackThreeAxis.Y);

        drivetrainSubsystem.westCoastDrive(throttleLeftValue, throttleRightValue, true);
    }

    /** At the end, stop the drivetrain. */
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.arcadeDrive(0.0, 0.0, false);
    }
}
