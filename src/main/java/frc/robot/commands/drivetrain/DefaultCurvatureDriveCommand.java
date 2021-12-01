package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.input.AttackThree;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultCurvatureDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final AttackThree leftStick;
    private final AttackThree rightStick;

    public DefaultCurvatureDriveCommand(RobotContainer robotContainer) {
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
        double rotationRightValue = 0.0;
        boolean squareQuickTurn;

        throttleLeftValue = leftStick.getY();
        rotationRightValue = rightStick.getX();

        squareQuickTurn = rightStick.getButton(3).get();
        drivetrainSubsystem.curvatureDrive(throttleLeftValue, rotationRightValue, squareQuickTurn);
    }

    /** At the end, stop the drivetrain. */
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.arcadeDrive(0.0, 0.0, false);
    }
}
