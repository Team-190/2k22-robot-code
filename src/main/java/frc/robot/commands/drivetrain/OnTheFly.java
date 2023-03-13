// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.HashMap;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class OnTheFly extends CommandBase {
  /** Creates a new OnTheFly. */
  RobotContainer robotContainer;
  DrivetrainSubsystem drivetrainSubsystem;
  PathPlannerTrajectory traj;
  RamseteController ramsete;
  HashMap<String, PathPoint> positions;
  PathConstraints constraints;
  String position;
  public OnTheFly(RobotContainer container, PathConstraints pathConstraints, String pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    robotContainer = container;
    drivetrainSubsystem = container.drivetrainSubsystem;
    ramsete = new RamseteController();
    ramsete.setEnabled(true);
    constraints = pathConstraints;
    position = pos;

    addRequirements(drivetrainSubsystem);

    positions = new HashMap<String, PathPoint>();
    positions.put("Pos7", new PathPoint(new Translation2d(1.9, 4.5), Rotation2d.fromDegrees(180)));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("goTo", "Run");
    drivetrainSubsystem.setOdometryAprilTag();

    traj = PathPlanner.generatePath(constraints, 
    new PathPoint(drivetrainSubsystem.getPose().getTranslation(), Rotation2d.fromDegrees(drivetrainSubsystem.navx.getAngle())),
    positions.get(position));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.westCoastDrive(0,0,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    (new PPRamseteCommand(traj,
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
                  true,
                  drivetrainSubsystem)).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
