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
        public static final int LEFT_FOLLOWER_CHANNEL = 2;
        public static final int RIGHT_LEADER_CHANNEL = 3;
        public static final int RIGHT_FOLLOWER_CHANNEL = 4;

        // PID Constants (Not Auto Constants)
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0;


        // Encoder and PID Constants (For Auto) // TODO: change for new robot
        public static final double TRACKWIDTH_METERS = 0.781987; // horizontal distance between wheels
        public static final double COUNTS_PER_MOTOR_REVOLUTION = 2048;
        public static final double WHEEL_DIAMETER_METERS = 0.1524;

        // 18 to 52 gear reduction 
        //TODO edit gear ratios
        public static final double WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS =
                (18.0 / 52.0) * (10.0 / 42.0);
        public static final double METERS_PER_COUNT =
                (1 / COUNTS_PER_MOTOR_REVOLUTION)
                        * // MOTOR ROTATIONS per count
                        WHEEL_REVOLUTIONS_PER_MOTOR_REVOLUTIONS
                        * (WHEEL_DIAMETER_METERS * Math.PI);


    }

    public static final class TurretConstants {

        public static final int TURRET_CHANNEL = 5;

        //PID Constants
        public static final int SLOT_ID = 0;
        public static final int PID_LOOPTYPE = 0;
        public static final int TIMEOUT_MS = 20;
        public static final double P = 20;
        public static final double I = 0;
        public static final double D = 0.5;
        public static final double F = 0;

        public static final double TURRET_STEP_SIZE = 1;



    }
}
