package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Superclass for auto trajectory lists.
 */
public abstract class Trajectories {

  public static TrajectoryConfig FORWARD_CONFIG = createTrajectoryConfig(false);
  public static TrajectoryConfig BACKWARD_CONFIG = createTrajectoryConfig(true);
  

  /**
   * Creates a new trajectory config
   *
   * @param reversed backwards or not
   * @return a new trajectory config
   */
  private static TrajectoryConfig createTrajectoryConfig(boolean reversed) {
    DrivetrainConstants.TRAJECTORY_CONFIG.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);
    DrivetrainConstants.TRAJECTORY_CONFIG.addConstraint(DrivetrainConstants.AUTO_VOLTAGE_CONSTRAINT);
    DrivetrainConstants.TRAJECTORY_CONFIG.setReversed(reversed);
    return DrivetrainConstants.TRAJECTORY_CONFIG;
  }
}