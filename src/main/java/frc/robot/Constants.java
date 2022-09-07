package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.util.Units;
import frc.robot.lightningUtil.swervelib.CanPort;

public final class Constants {

    public static final class DrivetrainConstants {

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.375);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.375);

        public static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
                SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final CanPort FRONT_LEFT_MODULE_DRIVE_MOTOR = new CanPort(8, "Canivore");
        public static final CanPort FRONT_LEFT_MODULE_STEER_MOTOR = new CanPort(7, "Canivore");
        public static final CanPort FRONT_LEFT_MODULE_STEER_ENCODER = new CanPort(16, "Canivore");
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.121);// -Math.toRadians(-95.09765625d);

        public static final CanPort FRONT_RIGHT_MODULE_DRIVE_MOTOR = new CanPort(11, "Canivore");
        public static final CanPort FRONT_RIGHT_MODULE_STEER_MOTOR = new CanPort(12, "Canivore");
        public static final CanPort FRONT_RIGHT_MODULE_STEER_ENCODER = new CanPort(17, "Canivore");
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(193.447);// -Math.toRadians(-12.744140625d);

        public static final CanPort BACK_LEFT_MODULE_DRIVE_MOTOR = new CanPort(10, "Canivore");
        public static final CanPort BACK_LEFT_MODULE_STEER_MOTOR = new CanPort(9, "Canivore");
        public static final CanPort BACK_LEFT_MODULE_STEER_ENCODER = new CanPort(15, "Canivore");
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(149.0625);// -Math.toRadians(30.673828125d);

        public static final CanPort BACK_RIGHT_MODULE_DRIVE_MOTOR = new CanPort(13, "Canivore");
        public static final CanPort BACK_RIGHT_MODULE_STEER_MOTOR = new CanPort(14, "Canivore");
        public static final CanPort BACK_RIGHT_MODULE_STEER_ENCODER = new CanPort(18, "Canivore");
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(59.765);// -Math.toRadians(119.00390625d);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.00057741;// 3.0801;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0.59292;
            public static final double kV = 2.7301;
            public static final double kA = 0.19945;
        }

        // Gains vaules for ProfiledPIDControllers
        public static final class ThetaGains {
            public static final double kP = 0.011334;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }
}