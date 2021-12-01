package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends PIDSubsystem {

    public final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER_CHANNEL);
    private final WPI_TalonFX leftFollower =
            new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER_CHANNEL);

    public final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER_CHANNEL);
    private final WPI_TalonFX rightFollower =
            new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_CHANNEL);

    private final SpeedControllerGroup leftSide = new SpeedControllerGroup(leftLeader, leftFollower);
    private final SpeedControllerGroup rightSide =
            new SpeedControllerGroup(rightLeader, rightFollower);

    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftSide, rightSide);

    // Objects for PID tracking
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry odometry =
            new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

    /**
    * Construct an instance of the Drivetrain
    *
    * @param kP The P value for the PIDF
    * @param ki The I value for the PIDF
    * @param kD The D value for the PIDF
    */
    public DrivetrainSubsystem(double kP, double ki, double kD) {

        super(new PIDController(kP, ki, kD));

        // Reset configuration to defaults
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        // Configure the Followers
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Configure the PID feedback and constants
        leftLeader.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                DrivetrainConstants.PID_LOOPTYPE,
                DrivetrainConstants.TIMEOUT_MS);
        rightLeader.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor,
                DrivetrainConstants.PID_LOOPTYPE,
                DrivetrainConstants.TIMEOUT_MS);

        configPIDF(
                leftLeader,
                DrivetrainConstants.P,
                DrivetrainConstants.I,
                DrivetrainConstants.D,
                DrivetrainConstants.F);
        configPIDF(
                rightLeader,
                DrivetrainConstants.P,
                DrivetrainConstants.I,
                DrivetrainConstants.D,
                DrivetrainConstants.F);

        // Wait for NAVX init before finishing DriveSubsystem init
        try {
            Thread.sleep(3000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Reset Drive Odometry
        odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(navx.getYaw()));
        setSetpoint(0);
    }

    @Override
    public void periodic() {
    }

    /**
    * Configures PIDF, not used by Trajectories
    *
    * @param motorController The motor controller to configure
    * @param P proportional value
    * @param I integral value
    * @param D derivative value
    * @param F feed forward value
    */
    public void configPIDF(WPI_TalonFX motorController, double P, double I, double D, double F) {
        motorController.config_kP(DrivetrainConstants.SLOT_ID, P);
        motorController.config_kI(DrivetrainConstants.SLOT_ID, I);
        motorController.config_kD(DrivetrainConstants.SLOT_ID, D);
        motorController.config_kF(DrivetrainConstants.SLOT_ID, F);
    }

    /**
    * Drive in Arcade mode
    *
    * @param throttle The Speed to go at
    * @param rotation The Rotation rate
    * @param square Whether to square the inputs
    */
    public void arcadeDrive(double throttle, double rotation, boolean square) {
        differentialDrive.arcadeDrive(throttle, rotation, square);
    }

    /**
    * Drive in WestCoast (Tank Drive mode)
    *
    * @param leftStick The Speed of the left side of the robot
    * @param rightStick The Speed of the right side of the robot
    * @param square Whether to square the inputs
    */
    public void westCoastDrive(double leftStick, double rightStick, boolean square) {
        differentialDrive.tankDrive(leftStick, rightStick, square);
    }

    /**
    * Drive in Curvature Drive (Ask Zach Boyer for an explanation)
    *
    * @param throttle The Speed to go at
    * @param radius The turning radius
    * @param quickTurn True to enable turn-in-place, False to disable
    */
    public void curvatureDrive(double throttle, double radius, boolean quickTurn) {
        differentialDrive.curvatureDrive(throttle, radius, quickTurn);
    }

    // PID methods

    @Override
    protected void useOutput(double output, double setpoint) {}

    @Override
    protected double getMeasurement() {
        return 0;
    }

    /**
    * Get the position of the robot relative to the starting position
    *
    * @return the position of the robot
    */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
}
