package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.lightningUtil.swervelib.CanPort;
import frc.lightningUtil.swervelib.SdsModuleConfigurations;

public final class Constants {

    public static final class DrivetrainConstants {

        // Our drivetrain and track width
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.375);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.375);

        // NavX angle offset
        public static final int STEER_OFFSET = 90;

        // Our max voltage and velocity
        public static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * // FIXME
                SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
                SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;
                
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 16;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.121);// -Math.toRadians(-95.09765625d);
    
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(195.447);
    
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 15;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(68.178); //
                                                                                            // -Math.toRadians(30.673828125d);
    
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 18;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(59.765);// 59.765);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 741;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0.59292;
            public static final double kV = 12; // 2.7301
            public static final double kA = 0.19945;
        }

        // Gains vaules for ProfiledPIDControllers
        public static final class ThetaGains {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }
}   