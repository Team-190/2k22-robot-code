package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.input.AttackThree;
import frc.robot.input.XboxOneController;
import frc.robot.input.AttackThree.AttackThreeAxis;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultArcadeDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final AttackThree leftStick;
    private final AttackThree rightStick;
    private final XboxOneController controller;

    public DefaultArcadeDriveCommand(RobotContainer robotContainer) {
        this.leftStick = robotContainer.leftStick;
        this.rightStick = robotContainer.rightStick;
        this.controller = robotContainer.driverXboxController;

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

        double throttleLeftValue;
        double rotationRightValue;

        throttleLeftValue = leftStick.getAxis(AttackThreeAxis.Y);
        rotationRightValue = rightStick.getAxis(AttackThreeAxis.X);
        // throttleLeftValue = leftStick.getY();
        // rotationRightValue = rightStick.getX();
  
        drivetrainSubsystem.arcadeDrive(throttleLeftValue, rotationRightValue, true);
    }

    /** At the end, stop the drivetrain. */
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.arcadeDrive(0.0, 0.0, false);
    }
}
