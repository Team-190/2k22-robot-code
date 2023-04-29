// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.collector.CollectCommand;
import frc.robot.commands.hotlineblink.BlinkinReadyToShootCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerFollowCommand extends SequentialCommandGroup {

  PathPlannerTrajectory autoPath;
  Map<String, Command> eventMap;
  RamseteAutoBuilder autoBuilder;
  List<PathPlannerTrajectory> autoGroup;
  RamseteController ramsete;

  public PathPlannerFollowCommand(RobotContainer robotContainer, boolean isFirstPath, String fileName) {

    autoGroup = PathPlanner.loadPathGroup(fileName, new PathConstraints(4, 3));

    eventMap = new HashMap<String,Command>();
    eventMap.put("CollectCommand", new CollectCommand(robotContainer, 0.75));
    eventMap.put("BlinkCommand", new BlinkinReadyToShootCommand(robotContainer));


    ramsete = new RamseteController();
    ramsete.setEnabled(true);

    autoBuilder = new RamseteAutoBuilder(
      robotContainer.drivetrainSubsystem::getPose,
      robotContainer.drivetrainSubsystem::resetOdometry,
      ramsete,
      new DifferentialDriveKinematics(DrivetrainConstants.TRACKWIDTH_METERS),
      new SimpleMotorFeedforward(
              DrivetrainConstants.S_VOLTS,
              DrivetrainConstants.V_VOLT_SECONDS_PER_METER,
              DrivetrainConstants.A_VOLT_SECONDS_SQUARED_PER_METER),
      robotContainer.drivetrainSubsystem::getWheelSpeeds,
      new PIDConstants(DrivetrainConstants.AUTO_P, 0, 0),
      robotContainer.drivetrainSubsystem::tankDriveVolts,
      eventMap,
      robotContainer.drivetrainSubsystem);
  
    addCommands(
      new InstantCommand(() -> {
      //Reset odometry for the first path
         if (isFirstPath) {
             robotContainer.drivetrainSubsystem.resetOdometry(autoGroup.get(0).getInitialPose());
           }}
      ),
      Commands.sequence(autoBuilder.fullAuto(autoGroup))
    );
  }
}
