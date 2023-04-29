package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Trajectory following class
 */
public class TrajectoryFollowerCommand extends RamseteCommand {
  private DrivetrainSubsystem drivetrainSubsystem;

  /**
   * Creates a Ramsete command that follows the given trajectory
   *
   * @param trajectory trajectory to follow
   */
  public TrajectoryFollowerCommand(RobotContainer robotContainer, Trajectory trajectory) {
    
    
    super(trajectory,
        robotContainer.drivetrainSubsystem::getPose,
        new RamseteController(DrivetrainConstants.RAMSETE_B, DrivetrainConstants.RAMSETE_ZETA),
        DrivetrainConstants.DRIVE_KINEMATICS,
        robotContainer.drivetrainSubsystem::tankDriveVolts,
        robotContainer.drivetrainSubsystem
    );
    
    
    /*
    super(
        trajectory, 
        robotContainer.drivetrainSubsystem::getPose,
        new RamseteController(DrivetrainConstants.RAMSETE_B, DrivetrainConstants.RAMSETE_ZETA),
        DrivetrainConstants.DRIVE_FEED_FORWARD,
        DrivetrainConstants.DRIVE_KINEMATICS,
        robotContainer.drivetrainSubsystem::getWheelSpeeds,
        new PIDController(DrivetrainConstants.AUTO_P, 0, 0),
        new PIDController(DrivetrainConstants.AUTO_P, 0, 0),
        // RamseteCommand passes volts to the callback
        robotContainer.drivetrainSubsystem::tankDriveVolts,
        robotContainer.drivetrainSubsystem
    );*/
    
    this.drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  /**
   * Starts preprogrammed trajectory code
   */
  @Override
  public void initialize() {
    super.initialize();
  }

  /**
   * Terminates preprogrammed trajectory code
   */
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrainSubsystem.tankDriveVolts(0, 0);
    //drivetrainSubsystem.invertDrivetrain(false);
  }
}
