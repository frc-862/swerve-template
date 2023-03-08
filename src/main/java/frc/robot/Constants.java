package frc.robot;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.util.Units;
import frc.thunder.swervelib.CanPort;
import frc.thunder.swervelib.SdsModuleConfigurations;

public final class Constants {

    // Constants for xbox controlers
    public static final class XboxControllerConstants {
        public static final double DEADBAND = 0.15;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;

    }

    public static final class LedConstants{
        public static final double BRIGHTNESS = .25;
        public static final LEDStripType STRIP_TYPE = LEDStripType.RGB;
        public static final int LED_LENGTH = 500;

        public static enum LEDStates {
            wantsCone, wantsCube, hasCone, hasCube, override, noPiece
        }
    }


    public static final class DrivetrainConstants {

        // Our drivetrain and track width
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.375);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.375);

        // Drivetrain PIDConstants
        // public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(Gains.kP,
        // Gains.kI, Gains.kD);
        // public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(ThetaGains.kP,
        // ThetaGains.kI,
        // ThetaGains.kD);

        // Stopped module constants
        public static final double FRONT_LEFT_RESTING_ANGLE = -45d;
        public static final double FRONT_RIGHT_RESTING_ANGLE = 45d;
        public static final double BACK_LEFT_RESTING_ANGLE = 45d;
        public static final double BACK_RIGHT_RESTING_ANGLE = -45d;

        // Our max voltage, velocity, and angular velocity
        public static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND =
                6380.0 / 60.0 * SdsModuleConfigurations.MK3_FAST.getDriveReduction()
                        * SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final int PIGEON = 1;
        public static final String PIGEON_CANBUS = "Canivore";

        // Front left moudle configurations
        public static final CanPort FRONT_LEFT_MODULE_DRIVE_MOTOR = new CanPort(8, "Canivore");
        public static final CanPort FRONT_LEFT_MODULE_STEER_MOTOR = new CanPort(7, "Canivore");
        public static final CanPort FRONT_LEFT_MODULE_STEER_ENCODER = new CanPort(16, "Canivore");
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(267.79449); // 267.97);

        // Front right moudle configurations
        public static final CanPort FRONT_RIGHT_MODULE_DRIVE_MOTOR = new CanPort(11, "Canivore");
        public static final CanPort FRONT_RIGHT_MODULE_STEER_MOTOR = new CanPort(12, "Canivore");
        public static final CanPort FRONT_RIGHT_MODULE_STEER_ENCODER = new CanPort(17, "Canivore");
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(195.46948); // 191.93);

        // Back left moudle configurations
        public static final CanPort BACK_LEFT_MODULE_DRIVE_MOTOR = new CanPort(10, "Canivore");
        public static final CanPort BACK_LEFT_MODULE_STEER_MOTOR = new CanPort(9, "Canivore");
        public static final CanPort BACK_LEFT_MODULE_STEER_ENCODER = new CanPort(15, "Canivore");
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(254.34998);// 254.11);

        // Back right moudle configurations
        public static final CanPort BACK_RIGHT_MODULE_DRIVE_MOTOR = new CanPort(13, "Canivore");
        public static final CanPort BACK_RIGHT_MODULE_STEER_MOTOR = new CanPort(14, "Canivore");
        public static final CanPort BACK_RIGHT_MODULE_STEER_ENCODER = new CanPort(18, "Canivore");
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(59.76288); // 59.08);

        // Gains vaules for PIDControllers
        public static final class Gains {
            public static final double kP = 0.15;
            public static final double kI = 0d;
            public static final double kD = 0d;

            public static final double kS = 0.59292;
            public static final double kV = 2.7301;
            public static final double kA = 0.19945;
        }

        // Gains vaules for ProfiledPIDControllers
        public static final class ThetaGains {
            public static final double kP = 4d;
            public static final double kI = 0d;
            public static final double kD = 0.05;

        }

    }

    public static final class VisionConstants {
        // Represents camera FOV from center to edge
        public static final double HORIZ_CAMERA_FOV = 29.8d;

        // Arbitrary value for how close the robot needs to be to the target (in angles)
        public static final double HORIZ_DEGREE_TOLERANCE = 3d;

        public static final Matrix<N3, N1> STANDARD_DEV_VISION_MATRIX =
                VecBuilder.fill(1.195384707229739, 0.7850610924749237, 2.2025094640913276);
    }
}
