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
    TrajectoryConfig CONFIG = new TrajectoryConfig(
                DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                DrivetrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
    );
    CONFIG.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);
    CONFIG.addConstraint(DrivetrainConstants.AUTO_VOLTAGE_CONSTRAINT);
    // CONFIG.setReversed(reversed);
    return CONFIG;
  }
}