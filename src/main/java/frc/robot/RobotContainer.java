/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Choosers
    public final SendableChooser<Integer> bottomRPMChooser = new SendableChooser<>();
    public final SendableChooser<Integer> topRPMChooser = new SendableChooser<>();

    /*
    * Subsystems
    */
    public final DrivetrainSubsystem drivetrainSubsystem =
            new DrivetrainSubsystem(
                    Constants.DrivetrainConstants.P,
                    Constants.DrivetrainConstants.I,
                    Constants.DrivetrainConstants.D);

        public final ShooterSubsystem shooterSubsystem = 
                new ShooterSubsystem();

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

        // Bottom RPM Chooser
        for (int i = 4000; i < 6001; i += 125) {
            bottomRPMChooser.addOption(""+i+ " RPM", i);
            topRPMChooser.addOption(""+i+" RPM", i);
        }

        SmartDashboard.putData("Bottom Wheel RPM", bottomRPMChooser);
        SmartDashboard.putData("Top Wheel RPM", topRPMChooser);

    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return null;
    }

    public void periodic() {
        SmartDashboard.putNumber("Bottom Wheel RPM Readout", shooterSubsystem.getBottomVelocity());
    }
}
