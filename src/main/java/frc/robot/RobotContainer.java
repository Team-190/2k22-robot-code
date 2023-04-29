/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Turret.TurretSetpointCommand;
import frc.robot.commands.drivetrain.OnTheFly;
import frc.robot.commands.hotlineblink.AllianceColorCommand;
import frc.robot.input.AttackThree;
import frc.robot.input.AttackThree.AttackThreeAxis;
import frc.robot.input.XboxOneController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

    // Choosers
    // public final SendableChooser<Integer> shooterRPMChooser = new SendableChooser<>();
    public final SendableChooser<Command> autoModeChooser = new SendableChooser<>();
    // public final SendableChooser<Integer> topRPMChooser = new SendableChooser<>();

    // Cameras
    UsbCamera camera1;

    /*
    * Subsystems
    */


    public final LimeLightSubsystem limeLightSubsystem =
            new LimeLightSubsystem();

     public final DrivetrainSubsystem drivetrainSubsystem =
            new DrivetrainSubsystem(
                    Constants.DrivetrainConstants.P,
                    Constants.DrivetrainConstants.I,
                    Constants.DrivetrainConstants.D,
                    this);        
    public final TurretSubsystem turretSubsystem =
            new TurretSubsystem(
                Constants.TurretConstants.P,
                Constants.TurretConstants.I,
                Constants.TurretConstants.D,
                limeLightSubsystem);

    public final HotlineBlinkSubsystem hotlineBlinkSubsystem = new HotlineBlinkSubsystem();

    public PathPoint initialPoint = new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0));


    
    // Compressor
    public final Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
    public boolean compressorEnabled = compressor.isEnabled();
    public boolean compressorPressureSwitch = compressor.getPressureSwitchValue();
    public double compressorCurrent = compressor.getCurrent();

    public final PowerDistribution pdh = new PowerDistribution(10, ModuleType.kRev);

    public final PneumaticHub pneumaticHub = new PneumaticHub(1);
    public final PneumaticsControlModule pcm = new PneumaticsControlModule();

    public double getX = 0;
    public double getY = 0;
    public double getRot = 0;


    /*
    * Input
    */
   public final AttackThree leftStick =
           new AttackThree(Constants.InputConstants.LEFT_JOYSTICK_CHANNEL);
    public final AttackThree rightStick =
            new AttackThree(Constants.InputConstants.RIGHT_JOYSTICK_CHANNEL);
    public final XboxOneController driverXboxController =
            new XboxOneController(Constants.InputConstants.XBOX_CHANNEL);


    /**
    * Constructor for the robot container Called when the Rio is powered on, and is only called once.
    * We use this to configure commands from buttons and default commands
    */
    PathPlannerTrajectory autoPath = PathPlanner.loadPath("Test_Path", new PathConstraints(2, 1));
    public RobotContainer() {
        /*
        for (int i = 1500; i < 6001; i += 50) {
            shooterRPMChooser.addOption(""+i+ " RPM", i);
        }
        */
        autoModeChooser.setDefaultOption("dD Nothing", new InstantCommand());
        autoModeChooser.addOption("AAHHHH", new RunCommand(()-> drivetrainSubsystem.westCoastDrive(0.2, 0.2, false)).withTimeout(2));


        SmartDashboard.putData("AutoModeChooser", autoModeChooser);
        SmartDashboard.putBoolean("Command Run", false);
        SmartDashboard.putString("CollectCommand", "Not Scheduled");
        SmartDashboard.putNumber("goToX", 0);
        SmartDashboard.putNumber("goToY", 0);
        SmartDashboard.putNumber("goToRotation", 0);
        SmartDashboard.putString("goTo", "Not Run");



        // SmartDashboard.putData("Set Flywheel RPM", shooterRPMChooser);

        // initializeCamera();

        // Left Stick Bindings


        // leftStick.triggerButton.whenPressed(new ToggleCollectorCommandGroup(this, 0.75));
        
        


        


        // Controller Bindings
        

        new POVButton(driverXboxController, 0)
            .whenPressed(new TurretSetpointCommand(this, 0));

        new POVButton(driverXboxController, 90)
            .whenPressed(new TurretSetpointCommand(this, turretSubsystem.degreesToTicks(90)));

        new POVButton(driverXboxController, 180)
            .whenPressed(new TurretSetpointCommand(this, turretSubsystem.degreesToTicks(180)));

        new POVButton(driverXboxController, 270)
            .whenPressed(new TurretSetpointCommand(this, turretSubsystem.degreesToTicks(-90)));

        driverXboxController.aButton.onTrue(new InstantCommand(()-> limeLightSubsystem.setPipeline(0)));

        driverXboxController.bButton.onTrue(new InstantCommand(()-> limeLightSubsystem.setPipeline(1)));

        driverXboxController.yButton.onTrue(new InstantCommand(()-> limeLightSubsystem.setPipeline(2)));

        rightStick.leftFaceButton.onTrue(new OnTheFly(this, 
             new PathConstraints(2, 1), "Pos7"));
        rightStick.rightFaceButton.onTrue(new InstantCommand(() -> drivetrainSubsystem.resetGyro(true)));



        //driverXboxController.leftBumper.whenPressed(new InstantCommand(()-> limeLightSubsystem.toggleVision()));

        
        
        // driverXboxController.yButton.whenPressed(new hoodToAngleCommand(this, 40));
        // driverXboxController.aButton.whenPressed(new hoodToAngleCommand(this, 27));

        /*
        new POVButton(driverXboxController, 0)
            .whileHeld(new RunCommand(()-> shooterSubsystem.relativeHoodAngle(0.5)));
            // .whenReleased(new InstantCommand(()-> shooterSubsystem.hoodManual(0)));

        new POVButton(driverXboxController, 180)
            .whileHeld(new RunCommand(()-> shooterSubsystem.relativeHoodAngle(-0.5)));
            // .whenReleased(new InstantCommand(()-> shooterSubsystem.hoodManual(0)));
        */
        


        drivetrainSubsystem.navx.calibrate();
        
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // return new PathPlannerFollowCommand(this, true, "Test_Path");
        //return new PathPlannerFollowCommand(this, false, "Pose7 Auto");
        //return drivetrainSubsystem.followTrajectoryCommand(autoPath, true);
        return autoModeChooser.getSelected();
        //  return new OnTheFly(this,
        //          new PathConstraints(1, 1), "Pos7");
    }

    public void setDefaultCommands() {
        // Default drive command
        //drivetrainSubsystem.setDefaultCommand(new DefaultTankDriveCommand(this));

        // Tank Joystick
         drivetrainSubsystem.setDefaultCommand(
             new RunCommand(
                 ()-> drivetrainSubsystem.arcadeDrive(leftStick.getAxis(AttackThreeAxis.Y), rightStick.getAxis(AttackThreeAxis.X), true), drivetrainSubsystem
             )
         );

        // Tank Controller
        // drivetrainSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         ()-> drivetrainSubsystem.westCoastDrive(driverXboxController.getLeftStickY(), driverXboxController.getRightStickY(), true), drivetrainSubsystem
        //     )
        // );

        // Arcade Joystick
        // drivetrainSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         ()-> drivetrainSubsystem.arcadeDrive(leftStick.getAxis(AttackThreeAxis.Y), rightStick.getAxis(AttackThreeAxis.X), true), drivetrainSubsystem
        //     )
        // );

        // Arcade Controller
        // drivetrainSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         ()-> drivetrainSubsystem.arcadeDrive(driverXboxController.getLeftStickY(), driverXboxController.getRightStickX(), false), drivetrainSubsystem
        //     )
        // );

        // Single Arcade Controller
        // drivetrainSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         ()-> drivetrainSubsystem.arcadeDrive(driverXboxController.getLeftStickY(), driverXboxController.getLeftStickX(), true), drivetrainSubsystem
        //     )
        // );

        hotlineBlinkSubsystem.setDefaultCommand(new AllianceColorCommand(this));


        // turretSubsystem.setDefaultCommand(new VisionCommand(this));

        // turretSubsystem.setDefaultCommand(new VisionCommand(this));
        //drivetrainSubsystem.setDefaultCommand(new DefaultArcadeDriveCommand(this));
        //climberSubsystem.setDefaultCommand(new ClimberJumpGrabCommand(this));

    }

    public void periodic() {
 
        initialPoint = new PathPoint(drivetrainSubsystem.getPose().getTranslation(), Rotation2d.fromDegrees(drivetrainSubsystem.navx.getAngle()));
    }

}
