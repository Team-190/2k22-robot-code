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
        public static final int LEFT_LEADER_CHANNEL = 9;
        public static final int LEFT_FOLLOWER_CHANNEL = 10;
        public static final int RIGHT_LEADER_CHANNEL = 13;
        public static final int RIGHT_FOLLOWER_CHANNEL = 12;

        // PID Constants (Not Auto Constants)
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;

        // Encoder and PID Constants (For Auto)
        public static final double TRACKWIDTH_METERS = 0.781987; // horizontal distance between wheels
        public static final double COUNTS_PER_MOTOR_REVOLUTION = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.1524;
        public static final double AUTO_P = 3.3514; // Calculated by SysID

        // 18 to 52 gear reduction 
        //TODO edit gear ratios
        public static final double WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS =
                (18.0 / 52.0) * (10.0 / 42.0);
        public static final double METERS_PER_COUNT =
                (1 / COUNTS_PER_MOTOR_REVOLUTION)
                        * // MOTOR ROTATIONS per count
                        WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS
                        * (WHEEL_DIAMETER_METERS * Math.PI);

        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACKWIDTH_METERS);
        public static final double RAMSETE_B = 2; 
        public static final double RAMSETE_ZETA = 0.7; // Was .9

        // Max Speed Constants
        public static final double MAX_SPEED_METERS_PER_SECOND = 2.7432; // Max speed set as 9 ft/s
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
        public final static int MAX_VOLTAGE = 11;

        // Constants calculated by System Identification software
        public static final double S_VOLTS = 0.74452; 
        public static final double V_VOLT_SECONDS_PER_METER = 2.8033;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.72842;

        public static final SimpleMotorFeedforward DRIVE_FEED_FORWARD =
                new SimpleMotorFeedforward(
                        DrivetrainConstants.S_VOLTS,
                        DrivetrainConstants.V_VOLT_SECONDS_PER_METER,
                        DrivetrainConstants.A_VOLT_SECONDS_SQUARED_PER_METER);

                        
        // Create a voltage constraint to ensure we don't accelerate too fast
        public final static DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
            DrivetrainConstants.DRIVE_FEED_FORWARD, DrivetrainConstants.DRIVE_KINEMATICS, DrivetrainConstants.MAX_VOLTAGE);
                
    

    }
    /** Constants for the Collector */
    public static final class CollectorConstants {

        // Motor CAN Id
        public static final int COLLECTOR_MOTOR_CHANNEL = 1;

        // Solenoid Ports
        public static final int COLLECTOR_SOLENOID_PORT_IN = 2;
        public static final int COLLECTOR_SOLENOID_PORT_OUT = 3;
    }
}
