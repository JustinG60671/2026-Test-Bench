package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Controllers {
        //Driver and Operator Contoller
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        //Controller
        public static final double DEADBAND = 0.08;
    }

     public interface Swerve {

        /** Proportional gain for the drive motor PID controller. */
        public static final double DRIVE_P = 0.23; // 0.39

        /** Integral gain for the drive motor PID controller. */
        public static final double DRIVE_I = 0.0; // 0.0001

        /** Derivative gain for the drive motor PID controller. */
        public static final double DRIVE_D = 0.0; // 0.00058

        /** Static feedforward gain for the drive motor. */
        public static final double DRIVE_S = 0.0;

        /** Velocity feedforward gain for the drive motor. */
        public static final double DRIVE_V = 0.12; // 1.2s

        /** Acceleration feedforward gain for the drive motor. */
        public static final double DRIVE_A = 0.0; // 0.01

        /** Motion Magic acceleration limit for drive motors (rotations per second squared). */
        public static final double DRIVE_MM_ACCELERATION = 350;

        /** Motion Magic jerk limiting for drive motors (rotations per second cubed). */
        public static final double DRIVE_MM_JERK = 4000;

        /** Proportional gain for the turn motor PID controller. */
        public static final double TURN_P = 60.0; // 56.5

        /** Integral gain for the turn motor PID controller. */
        public static final double TURN_I = 0.0; // 0.00000235

        /** Derivative gain for the turn motor PID controller. */
        public static final double TURN_D = 0.0;

        /** Static feedforward gain for the turn motor. */
        public static final double TURN_S = 0.5; // 0.42

        /** Velocity feedforward gain for the turn motor. */
        public static final double TURN_V = 0.0; // 0.008

        /** Acceleration feedforward gain for the turn motor. */
        public static final double TURN_A = 0.0;

        /** Motion Magic cruise velocity for turn motors (rotations per second). */
        public static final double TURN_MM_CRUISE_VELOCITY = 90; // 45

        /** Motion Magic acceleration limit for turn motors (rotations per second squared). */
        public static final double TURN_MM_ACCELERATION = 40; // 20

        /** Motion Magic jerk limiting for turn motors (rotations per second cubed). */
        public static final double TURN_MM_JERK = 3200; // 1500

        /** Diameter of the swerve module wheels in meters. */
        public static final double WHEEL_DIAMETER_METERS = 0.0889;

        /** Circumference of the swerve module wheels in meters. */
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        /** Gear reduction ratio for the driving motor. */
        public static final double DRIVING_MOTOR_REDUCTION = 4.67;

        /** Maximum linear speed of the robot in meters per second. */
        public static final double MAX_SPEED_METERS_PER_SECOND = 5.5;

        /** Maximum angular speed of the robot in radians per second. */
        public static final double MAX_ANGULAR_SPEED = 2.0 * Math.PI; // radians per second

        /**
         * Distance between left and right wheels in meters (measured between wheel
         * centers).
         */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23);

        /**
         * Distance between front and back wheels in meters (measured between wheel
         * centers).
         */
        public static final double WHEEL_BASE = Units.inchesToMeters(23);

        /** Kinematics object that defines the positions of all four swerve modules. */
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        /** Front left CANcoder absolute encoder offset in rotations. */
        public static final double FRONT_LEFT_MAGNET_OFFSET = 0.38525390625; // 0.38525390625

        /** Front right CANcoder absolute encoder offset in rotations. */
        public static final double FRONT_RIGHT_MAGNET_OFFSET = 0.10375976525; // 0.10375976525

        /** Back left CANcoder absolute encoder offset in rotations. */
        public static final double BACK_LEFT_MAGNET_OFFSET = 0.270751953125; // 0.270751953125

        /** Back right CANcoder absolute encoder offset in rotations. */
        public static final double BACK_RIGHT_MAGNET_OFFSET = -0.443359375; // -0.443359375

        /** CAN ID for the front left drive motor. */
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 2;

        /** CAN ID for the back left drive motor. */
        public static final int BACK_LEFT_DRIVING_CAN_ID = 0;

        /** CAN ID for the front right drive motor. */
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 3;

        /** CAN ID for the back right drive motor. */
        public static final int BACK_RIGHT_DRIVING_CAN_ID = 4;

        /** Inversion flag for front left drive motor. */
        public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;

        /** Inversion flag for back left drive motor. */
        public static final boolean BACK_LEFT_DRIVE_INVERTED = false;

        /** Inversion flag for front right drive motor. */
        public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;

        /** Inversion flag for back right drive motor. */
        public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;

        /** CAN ID for the front left turning motor. */
        public static final int FRONT_LEFT_TURNING_CAN_ID = 7;

        /** CAN ID for the back left turning motor. */
        public static final int BACK_LEFT_TURNING_CAN_ID = 6;

        /** CAN ID for the front right turning motor. */
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 1;

        /** CAN ID for the back right turning motor. */
        public static final int BACK_RIGHT_TURNING_CAN_ID = 5;

        /** CAN ID for the front left CANcoder absolute encoder. */
        public static final int FRONT_LEFT_CANCODER = 8;

        /** CAN ID for the front right CANcoder absolute encoder. */
        public static final int FRONT_RIGHT_CANCODER = 10;

        /** CAN ID for the back left CANcoder absolute encoder. */
        public static final int BACK_LEFT_CANCODER = 9;

        /** CAN ID for the back right CANcoder absolute encoder. */
        public static final int BACK_RIGHT_CANCODER = 11;

        /** CAN ID for the Pigeon 2 IMU. */
        public static final int PIGEON_CAN_ID = 12;

        /** Whether the gyro is reversed. */
        public static final boolean GYRO_REVERSED = false;

        /** Name of the CAN bus used for swerve drive motor communication. */
        public static final String CANBUS_NAME = "Swerve CAN Bus";

        /** Current limit for driving motors in amps. */
        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 60; // amps

        /** Current limit for turning motors in amps. */
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

        /**
         * Slow mode speed reduction factor to nerf the robot's maximum speed for better
         * control.
         */
        public static final double SLOW_INTAKE_NERF = 0.40;
        public static final double SLOW_DRIVE_NERF = 0.11;


    }
}
