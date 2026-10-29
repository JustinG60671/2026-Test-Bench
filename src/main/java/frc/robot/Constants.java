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

    }

    public interface Intake {
        static final int INTAKE_MOTOR_ID = 5;
        static final String CANBUS = "rio";
        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 40;

        static final int INTAKE2_MOTOR_ID = 5;
        public static final int INTAKE_MOTOR2_CURRENT_LIMIT = 40;
    }

    public interface Outtake {
        static final int OUTTAKE_MOTOR_ID = 6;
        static final String CANBUS = "rio";
        public static final int OUTTAKE_MOTOR_CURRENT_LIMIT = 40;
    }
}
