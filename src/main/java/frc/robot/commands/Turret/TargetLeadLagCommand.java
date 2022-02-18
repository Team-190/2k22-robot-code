// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TargetLeadLagCommand extends CommandBase {

  DrivetrainSubsystem drivetrainSubsystem = null;
  TurretSubsystem turretSubsystem = null;
  LimeLightSubsystem limeLightSubsystem = null;
  // ShooterSubsystem shooterSubsystem = null;

  double distance_target = 0;
  double magnitudeVelocity = 0;
  double angleToTarget = 0;
  double aimToAngle = 0;
  double xDistanceToLeadLagTarget = 0;
  double yDistanceToLeadLagTarget = 0;
  double magnitudeDistanceToLeadLagTarget = 0;

  /** Creates a new TargetLeadLagCommand. */
  public TargetLeadLagCommand(RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    this.turretSubsystem = robotContainer.turretSubsystem;
    this.limeLightSubsystem = robotContainer.limeLightSubsystem;
    // this.shooterSubsystem = robotContainer.shooterSubsystem;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ballVelocityProportional = 1; // shooterSubsystem.getBallVelocityProportional();
    double ballVelocityOffset = 1; // shooterSubsystem.getBallVelocityOffset();
    double hoodAngleProportional = 1; // shooterSubsystem.getHoodAngleProportional();
    double hoodAngleOffset = 1; // shooterSubsystem.getHoodAngleOffset();

    
    distance_target = limeLightSubsystem.getDistanceToTarget();
    magnitudeVelocity = drivetrainSubsystem.magnitudeVelocity();
    angleToTarget = limeLightSubsystem.degreesAskew() + turretSubsystem.getTurretAngle();
    xDistanceToLeadLagTarget = distance_target * Math.sin(Math.toRadians(angleToTarget));
    yDistanceToLeadLagTarget = (distance_target * Math.cos(Math.toRadians(angleToTarget))) - 
                              (
                                (magnitudeVelocity * distance_target) / 
                                  (
                                    ((ballVelocityProportional * distance_target) + ballVelocityOffset) *
                                    Math.cos(Math.toRadians(((hoodAngleProportional * distance_target) + hoodAngleOffset)))
                                  )
                              );
    magnitudeDistanceToLeadLagTarget = Math.sqrt(Math.pow(xDistanceToLeadLagTarget, 2) + Math.pow(yDistanceToLeadLagTarget, 2));
    // TODO: tell shooter to go to correct hood angle and flywheel speed using distnace to target
    // shooterSubsystem.setAngleAndSpeed(distance_target);
    turretSubsystem.turretPID( 
                              turretSubsystem.degreesToTicks( 
                                angleToTarget + Math.toDegrees(
                                  Math.acos(
                                    (
                                      (
                                        Math.pow(magnitudeVelocity, 2) - Math.pow(distance_target, 2) - Math.pow(magnitudeDistanceToLeadLagTarget, 2)
                                      ) 
                                      / 
                                      (
                                        -2 * magnitudeDistanceToLeadLagTarget * distance_target
                                      )
                                    )
                                  )
                                )
                              )
                            );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !limeLightSubsystem.targetFound() || turretSubsystem.turretUnwrap();
  }
}
