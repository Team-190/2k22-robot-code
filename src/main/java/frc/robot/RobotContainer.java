/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Turret.TurretSetpointCommand;
import frc.robot.commands.auto.threeBallAutoStraight.threeBallAutoStraight;
import frc.robot.commands.auto.twoBallAuto.twoBallAuto;
import frc.robot.commands.collector.AutomateBallpathCommand;
import frc.robot.commands.drivetrain.DefaultTankDriveCommand;
import frc.robot.commands.drivetrain.OnTheFlyPath;
import frc.robot.commands.hotlineblink.AllianceColorCommand;
import frc.robot.input.AttackThree;
import frc.robot.input.XboxOneController;
import frc.robot.input.AttackThree.AttackThreeAxis;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HotlineBlinkSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
   
    public final CollectorSubsystem collectorSubsystem = new CollectorSubsystem();

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

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public final HotlineBlinkSubsystem hotlineBlinkSubsystem = new HotlineBlinkSubsystem();

    
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
    //  public final ButtonBoxLeft buttonBoxLeft = new ButtonBoxLeft(2);
    // public final ButtonBoxRight buttonBoxRight = new ButtonBoxRight(3);


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
        autoModeChooser.setDefaultOption("2 Ball Auto", new twoBallAuto(this));
        autoModeChooser.addOption("3 Ball Straight Auto", new threeBallAutoStraight(this));


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

        leftStick.triggerButton.whenPressed(new InstantCommand(()-> collectorSubsystem.toggleCollector(.75), collectorSubsystem));
        leftStick.triggerButton.whenPressed(new AutomateBallpathCommand(this));

        // leftStick.triggerButton.whenPressed(new ToggleCollectorCommandGroup(this, 0.75));
        
        // Until interupt / timeout may fix stuff
        leftStick.middleFaceButton.whenPressed(new InstantCommand(()->  collectorSubsystem.collect(0.75)));
        leftStick.middleFaceButton.whenPressed(new AutomateBallpathCommand(this));

        leftStick.bottomFaceButton.whenHeld(new RunCommand(()-> collectorSubsystem.collect(-0.75)))
            .whenReleased(new InstantCommand(()-> collectorSubsystem.collect(0))); 

        
        rightStick.triggerButton.whenHeld(new RunCommand(()-> collectorSubsystem.upperBallPath(.7), collectorSubsystem))
            .whenReleased(new InstantCommand(()-> collectorSubsystem.upperBallPath(0), collectorSubsystem));


        


        // Controller Bindings
        driverXboxController.selectButton.whenPressed(new InstantCommand(()-> climberSubsystem.togglePivot()));
        

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

        // driverXboxController.yButton.onTrue(new InstantCommand(()-> limeLightSubsystem.setPipeline(2)));

         rightStick.leftFaceButton.onTrue(new OnTheFlyPath(this, 
             new PathConstraints(1, 1)));

        driverXboxController.xButton.whileTrue(new RunCommand(()-> drivetrainSubsystem.seekTarget(limeLightSubsystem, .25)));

        // driverXboxController.yButton.whileTrue(new RunCommand(()-> drivetrainSubsystem.goToDistance(60, limeLightSubsystem)));
        driverXboxController.yButton.whileTrue(new RunCommand(()-> drivetrainSubsystem.goAim(60, 1, 1, limeLightSubsystem)));





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
        //return new PathPlannerFollowCommand(this, true, "LOOP2");
        //return drivetrainSubsystem.followTrajectoryCommand(autoPath, true);
        return new OnTheFlyPath(this,
                 new PathConstraints(1, 1));
    }

    public void setDefaultCommands() {
        // Default drive command
        drivetrainSubsystem.setDefaultCommand(new DefaultTankDriveCommand(this));

        // Tank Joystick
         drivetrainSubsystem.setDefaultCommand(
             new RunCommand(
                 ()-> drivetrainSubsystem.westCoastDrive(leftStick.getAxis(AttackThreeAxis.X), rightStick.getAxis(AttackThreeAxis.Y), true), drivetrainSubsystem
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
        drivetrainSubsystem.setDefaultCommand(
            new RunCommand(
                ()-> drivetrainSubsystem.arcadeDrive(driverXboxController.getLeftStickY(), driverXboxController.getRightStickX(), false), drivetrainSubsystem
            )
        );

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
        if (!climberSubsystem.getTogglePivot() && turretSubsystem.getPivotTolerance()) {
            climberSubsystem.leftPivotActuate(true);
            climberSubsystem.rightPivotActuate(true);
        } else if (!climberSubsystem.getTogglePivot()) {
            climberSubsystem.leftPivotActuate(false);
            climberSubsystem.rightPivotActuate(false);
        }


        getX = SmartDashboard.getNumber("goToX", 0);
        getY = SmartDashboard.getNumber("goToY", 0);
        getRot = SmartDashboard.getNumber("goToRotation", 0);
    }

    /**
    * Initializes the camera(s)
    */
   public void initializeCamera() {
    try {

      // Intake
    //   camera1 = CameraServer.startAutomaticCapture();
    //   camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    //   camera1.setResolution(176, 144);
    //   camera1.setFPS(15); // Can go up to 30
    //   camera1.setBrightness(25);
    //   camera1.setExposureManual(10);
    //   camera1.setWhiteBalanceManual(10);
      
      /*
      // intake
      camera2 = CameraServer.startAutomaticCapture();
      camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera2.setResolution(176, 144);
      camera2.setFPS(15); // Can go up to 30
      camera2.setBrightness(25);
      camera2.setExposureManual(10);
      camera2.setWhiteBalanceManual(10);
      */

    } catch (VideoException e) {
      e.printStackTrace();
    }
   }
}
