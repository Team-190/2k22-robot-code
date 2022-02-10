package frc.robot.commands.auto.simpleTest;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.auto.Trajectories;

public class testTrajectory extends Trajectories{
    private static final Pose2d POINT1 = new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(180)));
    public static final Trajectory START = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0.000, 0.000, new Rotation2d(Units.degreesToRadians(180))),
            POINT1
        ),
        BACKWARD_CONFIG);
}
