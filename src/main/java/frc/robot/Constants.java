package frc.robot;

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
        public static final int LEFT_LEADER_CHANNEL = 1;
        public static final int LEFT_FOLLOWER_CHANNEL = 5;
        public static final int RIGHT_LEADER_CHANNEL = 6;
        public static final int RIGHT_FOLLOWER_CHANNEL = 4;

        // PID Constants (Not Auto Constants)
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;


    }

    /**
     * Constants for the shooterSubsystem
     */
    public static final class ShooterConstants {

        // CAN Ids
        public static final int BOTTOM_SHOOTER_CHANNEL = 2;
        public static final int TOP_SHOOTER_CHANNEL = 3;

        // shared PID Constants
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double TICKS_PER_ROTATION = 2048;
        public static final double MAX_SPEED_RPM = 6000;
        public static final int RPM_THRESHOLD = 50;

        // Bottom PID Constants
        public static final double BOTTOM_P = 0.00001;
        public static final double BOTTOM_I = 0.0005;
        public static final double BOTTOM_D = 0;
        public static final double BOTTOM_F = 0.050;

        // Top PID Constants
        public static final double TOP_P = 0.00001;
        public static final double TOP_I = 0.0005;
        public static final double TOP_D = 0;
        public static final double TOP_F = 0.050;


        /*
            0.03125 = 487.5 RPM
            0.045 = 750 RPM






         */

    }
}
