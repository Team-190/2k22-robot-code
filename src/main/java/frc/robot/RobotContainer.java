/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Turret.TurretSetpointCommand;
import frc.robot.commands.auto.simpleTest.testAuto;
import frc.robot.commands.drivetrain.DefaultTankDriveCommand;
import frc.robot.commands.shooter.hoodToAngleCommand;
import frc.robot.input.AttackThree;
import frc.robot.input.XboxOneController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
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
    public final SendableChooser<Integer> bottomRPMChooser = new SendableChooser<>();
    public final SendableChooser<Integer> topRPMChooser = new SendableChooser<>();

    // Cameras
    UsbCamera camera1;

    /*
    * Subsystems
    */
    public final DrivetrainSubsystem drivetrainSubsystem =
            new DrivetrainSubsystem(
                    Constants.DrivetrainConstants.P,
                    Constants.DrivetrainConstants.I,
                    Constants.DrivetrainConstants.D);
    public final CollectorSubsystem collectorSubsystem = new CollectorSubsystem();

    public final LimeLightSubsystem limeLightSubsystem =
            new LimeLightSubsystem();

    public final TurretSubsystem turretSubsystem =
            new TurretSubsystem(
                Constants.TurretConstants.P,
                Constants.TurretConstants.I,
                Constants.TurretConstants.D,
                limeLightSubsystem);

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final ShooterSubsystem shooterSubsystem = 
            new ShooterSubsystem();

    
    // Compressor
    public final Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    public boolean compressorEnabled = compressor.enabled();
    public boolean compressorPressureSwitch = compressor.getPressureSwitchValue();
    public double compressorCurrent = compressor.getCurrent();

    public final PowerDistribution pdh = new PowerDistribution(10, ModuleType.kRev);

    public final PneumaticHub pneumaticHub = new PneumaticHub(1);


    /*
    * Input
    */
   public final AttackThree leftStick =
           new AttackThree(Constants.InputConstants.LEFT_JOYSTICK_CHANNEL);
    public final AttackThree rightStick =
            new AttackThree(Constants.InputConstants.RIGHT_JOYSTICK_CHANNEL);
    public final XboxOneController driverXboxController =
            new XboxOneController(Constants.InputConstants.DRIVER_XBOX_CHANNEL);
    //  public final ButtonBoxLeft buttonBoxLeft = new ButtonBoxLeft(2);
    // public final ButtonBoxRight buttonBoxRight = new ButtonBoxRight(3);


    /**
    * Constructor for the robot container Called when the Rio is powered on, and is only called once.
    * We use this to configure commands from buttons and default commands
    */
    public RobotContainer() {

        // initializeCamera();

        // leftStick.triggerButton.whenHeld(new RunCommand(()-> collectorSubsystem.collect(.75), collectorSubsystem))
        //     .whenReleased(new RunCommand(()-> collectorSubsystem.collect(0), collectorSubsystem));

        // leftStick.triggerButton.whenPressed(new ToggleCollectCommand(this, 0.75));
        leftStick.triggerButton.whenPressed(new InstantCommand(()-> collectorSubsystem.toggleCollector(.75), collectorSubsystem));
        


        leftStick.middleFaceButton.whenPressed(new InstantCommand(()-> collectorSubsystem.extend(), collectorSubsystem));
        // leftStick.leftFaceButton.whenPressed(new InstantCommand(()-> turret.extend(), collectorSubsystem));

        // leftStick.leftFaceButton.whenPressed(new TurretSetpointCommand(this, -40000));
        // leftStick.rightFaceButton.whenPressed(new TurretSetpointCommand(this, 40000));
        // leftStick.bottomFaceButton.whenPressed(new TurretSetpointCommand(this, 0));

        // leftStick.leftFaceButton.whenPressed(new InstantCommand(()-> turretSubsystem.turretPID(-40000)));
        // leftStick.rightFaceButton.whenPressed(new InstantCommand(()-> turretSubsystem.turretPID(40000)));
        // leftStick.bottomFaceButton.whenPressed(new InstantCommand(()-> turretSubsystem.turretPID(0)));

        // leftStick.leftFaceButton.whenPressed(new RunCommand(()-> turretSubsystem.turretPID(-40000), turretSubsystem));
        // leftStick.rightFaceButton.whenPressed(new RunCommand(()-> turretSubsystem.turretPID(40000), turretSubsystem));
        // leftStick.bottomFaceButton.whenPressed(new RunCommand(()-> turretSubsystem.turretPID(0), turretSubsystem));

        leftStick.leftFaceButton.whenPressed(new TurretSetpointCommand(this, turretSubsystem.degreesToTicks(-90)));
        leftStick.rightFaceButton.whenPressed(new TurretSetpointCommand(this, turretSubsystem.degreesToTicks(90)));
        leftStick.bottomFaceButton.whenPressed(new TurretSetpointCommand(this, turretSubsystem.degreesToTicks(0)));

        rightStick.leftFaceButton.whenPressed(new RunCommand(()-> turretSubsystem.turretVision()));

        
        // leftStick.rightFaceButton.whenPressed(new InstantCommand(()-> collectorSubsystem.retract(), collectorSubsystem));

        rightStick.triggerButton.whenHeld(new RunCommand(()-> collectorSubsystem.upperBallPath(.7), collectorSubsystem))
            .whenReleased(new RunCommand(()-> collectorSubsystem.upperBallPath(0), collectorSubsystem));

        rightStick.middleFaceButton.whenPressed(new InstantCommand(()-> turretSubsystem.resetEncoder(0), turretSubsystem));

        rightStick.bottomFaceButton.whenPressed(new InstantCommand(()-> shooterSubsystem.flywheelToggle(3500)));
        // rightStick.bottomFaceButton.whenPressed(new ShootCommand(this, .55));

        driverXboxController.selectButton.whileHeld(new RunCommand(()-> turretSubsystem.turretManual(-0.2), turretSubsystem))
            .whenReleased(new InstantCommand(()-> turretSubsystem.turretManual(0), turretSubsystem));
        driverXboxController.startButton.whileHeld(new RunCommand(()-> turretSubsystem.turretManual(0.2), turretSubsystem))
            .whenReleased(new InstantCommand(()-> turretSubsystem.turretManual(0), turretSubsystem));

        // buttonBoxLeft.traverseLeft.whileHeld(new RunCommand(()-> climberSubsystem.extendClimber(.6), climberSubsystem))
        //     .whenReleased(new RunCommand(()-> climberSubsystem.extendClimber(0), climberSubsystem));
        // buttonBoxRight.traverseRight.whileHeld(new RunCommand(()-> climberSubsystem.extendClimber(-.6), climberSubsystem))

        new POVButton(driverXboxController, 90)
            .whenHeld(new RunCommand(()-> turretSubsystem.relativeTurretPID(turretSubsystem.degreesToTicks(.5)), turretSubsystem));

        new POVButton(driverXboxController, 270)
            .whenHeld(new RunCommand(()-> turretSubsystem.relativeTurretPID(-turretSubsystem.degreesToTicks(.5)), turretSubsystem));

        
        /*
        new POVButton(driverXboxController, 0)
            .whileHeld(new RunCommand(()-> climberSubsystem.extendClimber(.3), climberSubsystem))
            .whenReleased(new RunCommand(()-> climberSubsystem.extendClimber(0), climberSubsystem));

        new POVButton(driverXboxController, 180)
            .whileHeld(new RunCommand(()-> climberSubsystem.extendClimber(-.3), climberSubsystem))
            .whenReleased(new RunCommand(()-> climberSubsystem.extendClimber(0), climberSubsystem));
        */
        

        /*
        driverXboxController.yButton.whenPressed(new ClimbUpCommand(this));
        driverXboxController.aButton.whenPressed(new ClimberDownCommand(this));
        */

        driverXboxController.yButton.whenPressed(new hoodToAngleCommand(this, 40));
        driverXboxController.aButton.whenPressed(new hoodToAngleCommand(this, 27));

        new POVButton(driverXboxController, 0)
            .whileHeld(new RunCommand(()-> shooterSubsystem.relativeHoodAngle(0.5)));
            // .whenReleased(new InstantCommand(()-> shooterSubsystem.hoodManual(0)));

        new POVButton(driverXboxController, 180)
            .whileHeld(new RunCommand(()-> shooterSubsystem.relativeHoodAngle(-0.5)));
            // .whenReleased(new InstantCommand(()-> shooterSubsystem.hoodManual(0)));

        rightStick.rightFaceButton.whenPressed(new InstantCommand(()-> shooterSubsystem.resetHood(27)));

        // new Trigger(()-> (Math.abs(driverXboxController.getLeftStickY()) > 0.05))
        //     .whenActive(new RunCommand(()-> shooterSubsystem.hoodManual(driverXboxController.getLeftStickY())))
        //     .whenInactive(new InstantCommand(()-> shooterSubsystem.relativeHoodAngle(0)));

        new Trigger(()-> (driverXboxController.getRightTrigger() > 0.005))
            .whenActive(new RunCommand(()-> climberSubsystem.brakeActuate(true), climberSubsystem))
            .whenInactive(new RunCommand(()-> climberSubsystem.brakeActuate(false), climberSubsystem));

        // buttonBoxRight.colorWheel.whenPressed(new RunCommand(()-> climberSubsystem.brakeActuate(true), climberSubsystem));
        // buttonBoxRight.colorWheel.whenReleased(new RunCommand(()-> climberSubsystem.brakeActuate(false), climberSubsystem));

        
        driverXboxController.rightBumper.whenPressed(new InstantCommand(()-> climberSubsystem.clamperToggle()));



        driverXboxController.leftBumper.whenPressed(new InstantCommand(()-> limeLightSubsystem.toggleVision()));
        // buttonBoxRight.release.whenPressed(new InstantCommand(()-> turretSubsystem.resetEncoder(-40000)));
        
        // buttonBoxRight.tilt.whenPressed(new InstantCommand(()-> turretSubsystem.resetEncoder(40000)));

        driverXboxController.bButton.whenPressed(new InstantCommand(()-> climberSubsystem.jumperActuate(true)));
        

        new Trigger(()-> (driverXboxController.getLeftTrigger() > 0.005))
            .whenActive(new RunCommand(()-> collectorSubsystem.upperBallPath(-.7), collectorSubsystem))
            .whenInactive(new RunCommand(()-> collectorSubsystem.upperBallPath(0), collectorSubsystem));

        drivetrainSubsystem.gyro.calibrate();


        /*
        //TODO 
        new JoystickButton(driverXboxController, 5)
        .whenPressed(new InstantCommand(()-> turretSubsystem.turretPID(-25), turretSubsystem))
        .whenReleased(new InstantCommand(()-> turretSubsystem.turretManual(0), turretSubsystem));

        new JoystickButton(driverXboxController, 6)
        .whenPressed(new InstantCommand(()-> turretSubsystem.turretPID(25), turretSubsystem))
        .whenReleased(new InstantCommand(()-> turretSubsystem.turretManual(0), turretSubsystem));

        new JoystickButton(driverXboxController, 2)
        .whenPressed(new InstantCommand(()-> turretSubsystem.turretManual(.11), turretSubsystem))
        .whenReleased(new InstantCommand(()-> turretSubsystem.turretManual(0), turretSubsystem));

        new JoystickButton(driverXboxController, 3)
        .whenPressed(new InstantCommand(()-> turretSubsystem.turretManual(-.11), turretSubsystem))
        .whenReleased(new InstantCommand(()-> turretSubsystem.turretManual(0), turretSubsystem));

        new JoystickButton(driverXboxController, 4)
                .whenPressed(new InstantCommand(() -> turretSubsystem.resetEncoder(), turretSubsystem));

        new JoystickButton(driverXboxController, 1)
                .whenPressed(new VisionCommand(this));


        // driverXboxController.aButton.whenPressed(new InstantCommand(()-> drivetrainSubsystem.resetAll(), drivetrainSubsystem));
        //leftStick.middleFaceButton.whenPressed(new InstantCommand(()-> drivetrainSubsystem.resetGyro(false), drivetrainSubsystem));
        
        
        rightStick.middleFaceButton.whenPressed(() -> {
            new ClimberJumpGrabCommand(this).schedule();
        });
        */
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // return null;
        return new testAuto(this);
    }

    public void setDefaultCommands() {
        // Default drive command
        drivetrainSubsystem.setDefaultCommand(new DefaultTankDriveCommand(this));
        // turretSubsystem.setDefaultCommand(new VisionCommand(this));
        //drivetrainSubsystem.setDefaultCommand(new DefaultArcadeDriveCommand(this));
        //climberSubsystem.setDefaultCommand(new ClimberJumpGrabCommand(this));

    }

    public void periodic() {
        /*
        if (!climberSubsystem.jumperLimitSwitch.get()) {
            new ClimberJumpGrabCommand(this).schedule();
        }
        */

    }

    /**
    * Initializes the camera(s)
    */
   public void initializeCamera() {
    try {

      // Intake
      camera1 = CameraServer.startAutomaticCapture();
      camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera1.setResolution(176, 144);
      camera1.setFPS(15); // Can go up to 30
      camera1.setBrightness(25);
      camera1.setExposureManual(10);
      camera1.setWhiteBalanceManual(10);
      
      /*
      // intake
      camera2 = CameraServer.getInstance().startAutomaticCapture();
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
