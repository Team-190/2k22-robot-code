// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnTheFlyPath extends SequentialCommandGroup {
  /** Creates a new OnTheFlyPath. */
  RobotContainer robotContainer;
  DrivetrainSubsystem drivetrainSubsystem;
  PathPlannerTrajectory traj;
  RamseteController ramsete;
  double x;
  double y;
  double rotationDegrees;
  public OnTheFlyPath(RobotContainer container, PathConstraints constraints) {
    // Use addRequirements() here to declare subsystem dependencies.
    robotContainer = container;
    drivetrainSubsystem = container.drivetrainSubsystem;

    ramsete = new RamseteController();
    ramsete.setEnabled(true);
    
    addRequirements(drivetrainSubsystem);

    addCommands(new InstantCommand(() -> {
        SmartDashboard.putString("goTo", "Run");
        drivetrainSubsystem.setOdometryAprilTag();
      }),
      new ParallelCommandGroup(new InstantCommand(() -> SmartDashboard.putString("goTo", "Attempting Run")),
      new PPRamseteCommand(
        PathPlanner.generatePath(constraints, 
        new PathPoint(drivetrainSubsystem.getPose().getTranslation(), Rotation2d.fromDegrees(drivetrainSubsystem.navx.getAngle())),
        new PathPoint(new Translation2d(SmartDashboard.getNumber("goToX", 0),SmartDashboard.getNumber("goToY", 0)), new Rotation2d(SmartDashboard.getNumber("goToRotation", 0)))),
                    drivetrainSubsystem::getPose,
                    //new RamseteController(DrivetrainConstants.RAMSETE_B, DrivetrainConstants.RAMSETE_ZETA),
                    ramsete,
                    new SimpleMotorFeedforward(
                            DrivetrainConstants.S_VOLTS,
                            DrivetrainConstants.V_VOLT_SECONDS_PER_METER,
                            DrivetrainConstants.A_VOLT_SECONDS_SQUARED_PER_METER),
                    new DifferentialDriveKinematics(DrivetrainConstants.TRACKWIDTH_METERS),
                    drivetrainSubsystem::getWheelSpeeds,
                    new PIDController(DrivetrainConstants.AUTO_P, 0, 0),
                    new PIDController(DrivetrainConstants.AUTO_P, 0, 0),
                    // RamseteCommand passes volts to the callback
                    drivetrainSubsystem::tankDriveVolts,
                    drivetrainSubsystem))

    );
  }
}
