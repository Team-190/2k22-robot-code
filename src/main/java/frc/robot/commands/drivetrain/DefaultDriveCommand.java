// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants.DRIVE_INPUT;
import frc.robot.Constants.DrivetrainConstants.DRIVE_STYLE;
import frc.robot.input.AttackThree;
import frc.robot.input.XboxOneController;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final RobotContainer robotContainer;

  private final AttackThree leftStick;
  private final AttackThree rightStick;
  private final XboxOneController xboxController;
  private static DRIVE_STYLE drivestyle;
  private static DRIVE_INPUT driveInput;

  public DefaultDriveCommand(RobotContainer robotContainer) {
      this.leftStick = robotContainer.leftStick;
      this.rightStick = robotContainer.rightStick;
      this.xboxController = robotContainer.driverXboxController;
      drivestyle = robotContainer.driveStyleChooser.getSelected();
      driveInput = robotContainer.driveInputChooser.getSelected();
      this.drivetrainSubsystem = robotContainer.drivetrainSubsystem;
      this.robotContainer = robotContainer;

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

      drivestyle = robotContainer.driveStyleChooser.getSelected();
      driveInput = robotContainer.driveInputChooser.getSelected();
      double throttleLeftValue = 0.0;
      double rotationRightValue = 0.0;
      double throttleRightValue = 0.0;
      boolean square = false;
      square = robotContainer.squareChooser.getSelected();
    
      if (driveInput == DRIVE_INPUT.JOYSTICKS) {
        throttleLeftValue = -leftStick.getY();
        rotationRightValue = rightStick.getX();
        throttleRightValue = -rightStick.getY();
      } else if (driveInput == DRIVE_INPUT.CONTROLLER) {
        throttleLeftValue = xboxController.getLeftStickY();
        rotationRightValue = xboxController.getRightStickX();
        throttleRightValue = xboxController.getRightStickY();
      }

      if (drivestyle == DRIVE_STYLE.MCFLY) {
        if (driveInput == DRIVE_INPUT.JOYSTICKS) {
        square = rightStick.getRawButton(2);
        } else if (driveInput == DRIVE_INPUT.JOYSTICKS) {
          square = xboxController.getLeftBumper();
        }
        drivetrainSubsystem.curvatureDrive(throttleLeftValue, rotationRightValue, square);
      } else {
        if (drivestyle == DRIVE_STYLE.TANK) {
          drivetrainSubsystem.westCoastDrive(throttleLeftValue, throttleRightValue, square);
        } else if (drivestyle == DRIVE_STYLE.ARCADE) {
          drivetrainSubsystem.arcadeDrive(throttleLeftValue, rotationRightValue, square);
        }
      }
      
  }

  /** At the end, stop the drivetrain. */
  @Override
  public void end(boolean interrupted) {
      drivetrainSubsystem.westCoastDrive(0.0, 0.0, false);
  }
}