/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.DefaultArcadeDriveCommand;
import frc.robot.input.AttackThree;
import frc.robot.input.XboxOneController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    /*
    * Subsystems
    */
    public final DrivetrainSubsystem drivetrainSubsystem =
            new DrivetrainSubsystem(
                    Constants.DrivetrainConstants.P,
                    Constants.DrivetrainConstants.I,
                    Constants.DrivetrainConstants.D);

        public final ShooterSubsystem shooterSubsystem = 
                new ShooterSubsystem(
                Constants.ShooterConstants.P,
                Constants.ShooterConstants.I,
                Constants.ShooterConstants.D, 
                Constants.ShooterConstants.F);

    /*
    * Input
    */
    public final AttackThree leftStick =
            new AttackThree(Constants.InputConstants.LEFT_JOYSTICK_CHANNEL);
    public final AttackThree rightStick =
            new AttackThree(Constants.InputConstants.RIGHT_JOYSTICK_CHANNEL);
    public final XboxOneController driverXboxController =
            new XboxOneController(Constants.InputConstants.DRIVER_XBOX_CHANNEL);

    /**
    * Constructor for the robot container Called when the Rio is powered on, and is only called once.
    * We use this to configure commands from buttons and default commands
    */
    public RobotContainer() {
        
        driverXboxController.aButton.whenPressed(new InstantCommand(() -> shooterSubsystem.shooterManual(0.5), shooterSubsystem))
                .whenReleased(new InstantCommand(() -> shooterSubsystem.shooterManual(0), shooterSubsystem));

        driverXboxController.bButton.whenPressed(new InstantCommand(() -> shooterSubsystem.shooterPID(0), shooterSubsystem));

    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return null;
    }

    public void setDefaultCommands() {
        // Default drive command
        drivetrainSubsystem.setDefaultCommand(new DefaultArcadeDriveCommand(this));
    }

    public void periodic() {}
}
