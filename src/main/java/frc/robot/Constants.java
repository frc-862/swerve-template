package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.lightningUtil.swervelib.CanPort;

public final class Constants {

    public static final double REAL_MAX_SPEED = 16.2; // ft/sec
    public static final double MAX_SPEED = REAL_MAX_SPEED; // ft/sec
    public static final double MAX_ANGULAR_SPEED = MAX_SPEED; // ft/sec
    public static final double MAX_ANGULAR_ACCEL = 2 * Math.PI;
    public static final double MAX_ACCEL = 10d; // ft/sec^2
    public static final double MAX_VOLTAGE = 12d;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.375);

    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.375);
    
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
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(68.178);

    // Back right module
    public static final CanPort BACK_RIGHT_MODULE_DRIVE_MOTOR = new CanPort(13, "Canivore");
    public static final CanPort BACK_RIGHT_MODULE_STEER_MOTOR = new CanPort(14, "Canivore");
    public static final CanPort BACK_RIGHT_MODULE_STEER_ENCODER = new CanPort(18, "Canivore");
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(59.765);

}
