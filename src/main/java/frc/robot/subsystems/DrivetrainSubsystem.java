package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends PIDSubsystem {

    public final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER_CHANNEL);
    private final WPI_TalonFX leftFollower =
            new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER_CHANNEL);

    public final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER_CHANNEL);
    private final WPI_TalonFX rightFollower =
            new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER_CHANNEL);

    private final MotorControllerGroup leftSide = new MotorControllerGroup(leftLeader, leftFollower);
    private final MotorControllerGroup rightSide = new MotorControllerGroup(rightLeader, rightFollower);

    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftSide, rightSide);

    // Objects for PID tracking
    // private final AHRS navx = new AHRS(SPI.Port.kMXP);
    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
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

        invertDrivetrain(false);

        // invertDrivetrain(true);
        

        // Set Break Mode
        setBreakMode();

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

        // Wait for Gyro init before finishing DriveSubsystem init
        try {
            Thread.sleep(5000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Reset Drive Odometry, Encoders, and Gyro
        resetAll();
        setSetpoint(0);
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Left Drive Encoder", leftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Drive Encoder", rightLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("Get left wheel speed", leftLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Get right wheel speed", rightLeader.getSelectedSensorVelocity());
        SmartDashboard.putNumber("gyro raw yaw", gyro.getAngle());
        SmartDashboard.putNumber("gyro yaw", getYawDegrees());
        
        

        // Update the Odometry
        odometry.update(
            Rotation2d.fromDegrees(getYawDegrees()),
            getDistanceMeters(leftLeader),
            getDistanceMeters(rightLeader)
        );
        
        
    }


    /**
     * Gets distance in meters
     *
     * @return the distance in meters
     */
    public double getDistanceMeters(TalonFX talon) {
        return -talon.getSelectedSensorPosition() * DrivetrainConstants.METERS_PER_COUNT;
    }

    /**
     * (counts / 100 ms) * (meters / count) * (10 ms / 1 s) == (meters / second)
     *
     * @return Wheel speeds in meters / second
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            -leftLeader.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_COUNT * 10,
            -rightLeader.getSelectedSensorVelocity() * DrivetrainConstants.METERS_PER_COUNT * 10
        );
    }

    

    /**
     * Gets the chassis's yaw (orientation of the robot)
     *
     * @return yaw in degrees
     */
    public double getYawDegrees() { // -180 to 180 degrees
        double angle = (gyro.getAngle()*7.5) % 360;
        if (angle <= 180.0)
            return angle;
        return angle - 360;
    }

    /**
     * Sets drive motors to brake
     */
    public void setBreakMode() {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets drive motors to coast
     */
    public void setCoastMode() {
        leftLeader.setNeutralMode(NeutralMode.Coast);
        rightLeader.setNeutralMode(NeutralMode.Coast);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        rightFollower.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Invert Drivetrain motors
     */
    public void invertDrivetrain(boolean reversed) {
        if (reversed) {
            leftLeader.setInverted(TalonFXInvertType.Clockwise);
            leftFollower.setInverted(TalonFXInvertType.FollowMaster);
            rightLeader.setInverted(TalonFXInvertType.CounterClockwise);
            rightFollower.setInverted(TalonFXInvertType.FollowMaster);
        } else {
            leftLeader.setInverted(TalonFXInvertType.CounterClockwise);
            leftFollower.setInverted(TalonFXInvertType.FollowMaster);
            rightLeader.setInverted(TalonFXInvertType.Clockwise);
            rightFollower.setInverted(TalonFXInvertType.FollowMaster);
        }

        
        
    }

    /**
     * Sets encoders to 0
     */
    private void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    /**
     * Resets the Yaw position of the gyro
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Resets odometry to specified pose
     *
     * @param pose pose to reset to
     */
    private void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getYawDegrees()));
    }

    /**
     * Resets gyro, Encoders, odometry
     */
    public void resetAll() {
        resetGyro();
        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(getYawDegrees())));
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
        //differentialDrive.tankDrive(Math.copySign(Math.pow(leftStick, power), leftStick), Math.copySign(Math.pow(leftStick, power), leftStick));
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

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
        differentialDrive.feed();
    }


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
