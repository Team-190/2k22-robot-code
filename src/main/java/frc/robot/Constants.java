package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class Constants {

    /** Constants for the Inputs, like Attack 3s and Xbox Controllers */
    public static final class InputConstants {

        public enum INPUT_METHOD {
            CONTROLLER,
            STICKS,
            KINECT,
            BUTTON_BOX
        }

        // USB Ids
        public static final int LEFT_JOYSTICK_CHANNEL = 0;
        public static final int RIGHT_JOYSTICK_CHANNEL = 1;
        public static final int DRIVER_XBOX_CHANNEL = 2;
        public static final int OPERATOR_XBOX_CHANNEL = 3;
        public static final int BUTTON_BOX_LEFT_CHANNEL = 4;
        public static final int BUTTON_BOX_RIGHT_CHANNEL = 5;
    }

    public static final class CollectorConstants {
        // CAN Ids
        public static final int UPPERBALLPATH_CHANNEL = 6;
        public static final int COLLECTOR_CHANNEL = 5;

        // Solenoid Ids
        public static final int FORWARD_CHANNEL = 4;
        public static final int REVERSE_CHANNEL = 5;



    }

    /**
     * Constants for the drivetrainSubsystem
     */
    public static final class DrivetrainConstants {

        public enum DRIVE_STYLE {
            ARCADE,
            TANK,
            MCFLY // Curvature
        }

        // CAN Ids
        public static final int LEFT_LEADER_CHANNEL = 2;
        public static final int LEFT_FOLLOWER_CHANNEL = 3;
        public static final int RIGHT_LEADER_CHANNEL = 4;
        public static final int RIGHT_FOLLOWER_CHANNEL = 5;

        // PID Constants (Not Auto Constants)
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;

        // Encoder and PID Constants (For Auto)
        public static final double TRACKWIDTH_METERS = 0.7144; // horizontal distance between wheels
        public static final double COUNTS_PER_MOTOR_REVOLUTION = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inch diameter in meters
        public static final double AUTO_P = 2.8055; // Calculated by SysID

        // (14/58) ratio to (20/28) on the drivetrain gearbox
        public static final double WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS = 0.172; // (0.172)
        public static final double METERS_PER_COUNT =
                (1 / COUNTS_PER_MOTOR_REVOLUTION)
                        * // MOTOR ROTATIONS per count
                        WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS
                        * (WHEEL_DIAMETER_METERS * Math.PI);

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACKWIDTH_METERS);
        public static final double RAMSETE_B = 2; 
        public static final double RAMSETE_ZETA = 0.7;

        // Max Speed Constants
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.7432; // Max speed set as 9 ft/s
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
        public final static int MAX_VOLTAGE = 11;

        // Constants calculated by System Identification software
        public static final double S_VOLTS = 0.70419; 
        public static final double V_VOLT_SECONDS_PER_METER = 1.9284;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.4825;

        public static final SimpleMotorFeedforward DRIVE_FEED_FORWARD =
                new SimpleMotorFeedforward(
                        DrivetrainConstants.S_VOLTS,
                        DrivetrainConstants.V_VOLT_SECONDS_PER_METER,
                        DrivetrainConstants.A_VOLT_SECONDS_SQUARED_PER_METER);

                        
        // Create a voltage constraint to ensure we don't accelerate too fast
        public final static DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
            DrivetrainConstants.DRIVE_FEED_FORWARD, DrivetrainConstants.DRIVE_KINEMATICS, DrivetrainConstants.MAX_VOLTAGE);
                
    }

    public static final class TurretConstants {

        public static final int TURRET_CHANNEL = 7;

        //PID Constants
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = .06;
        public static final double I = 0;
        public static final double D = 0.0;
        public static final int TOLERANCE = 100;
        public static final int TICKS_PER_ROTATION = 2048;
        public static final double TURRET_GEAR_RATIO = 0.012987012987; // 77:1 gear ratio
        public static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * 1/TURRET_GEAR_RATIO) / 360;

        public static final double TURRET_STEP_SIZE = 1;
        public static final int TURRET_MOTOR_VELOCITY = 6380;
        public static final int TURRET_MOTOR_ACCELERATION = TURRET_MOTOR_VELOCITY * 2;
        public static final int TURRET_MOTOR_MOTION_SMOOTHING = 8;




    }

    public static final class ClimberConstants {

        public static final int JUMPER_ID = 10;
        public static final int RELEASE_JUMPER_ID = 1;
        public static final int CLAMPER_ID = 2;
        public static final int PIVOT_ID = 5;
        public static final int EXTENDER_ID = 4;
        public static final int BREAK_ID = 3;

        public static final int CLIMBER_MOTOR_CHANNEL = 9;
    }

    /**
     * Constants for the shooterSubsystem
     */
    public static final class ShooterConstants {

        // CAN Ids
        public static final int FLYWHEEL_CHANNEL = 8;
        //public static final int HOOD_CHANNEL = 6;

        // Shared PID Values
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double TICKS_PER_ROTATION = 2048;

        // Flywheel PID Constants
        public static final double FLYWHEEL_P = 0.0;
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.0;
        public static final double MAX_SPEED_RPM = 6380;
        public static final int RPM_THRESHOLD = 50;

        // Hood PID Constants
        public static final double HOOD_P = 0.0;
        public static final double HOOD_I = 0.0;
        public static final double HOOD_D = 0.0;
        public static final int HOOD_TOLERANCE = 10;

        

    }
}
