package frc.robot.commands.auto.twoBallAuto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.auto.Trajectories;

public class twoBallTrajectory extends Trajectories{
    private static final Pose2d POINT1 = new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0)));
    private static final Pose2d POINT2 = new Pose2d(2, -1, new Rotation2d(Units.degreesToRadians(-90)));

    public static final Trajectory START = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(0.000, 0.000, new Rotation2d(Units.degreesToRadians(0))),
            POINT1
        ),
        FORWARD_CONFIG);

    public static final Trajectory TURN = TrajectoryGenerator.generateTrajectory(
        List.of(
            POINT1,
            POINT2
        ),
        FORWARD_CONFIG);
    
}
