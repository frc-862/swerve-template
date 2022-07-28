package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final double REAL_MAX_SPEED = 16.2; // ft/sec
    public static final double MAX_SPEED = REAL_MAX_SPEED; // ft/sec
    public static final double MAX_ANGULAR_SPEED = MAX_SPEED; // ft/sec
    public static final double MAX_ANGULAR_ACCEL = 2 * Math.PI;
    public static final double MAX_ACCEL = 10d; // ft/sec^2
    public static final double MAX_VOLTAGE = 12d;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.375);
    
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.375);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 16;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.121);// -Math.toRadians(-95.09765625d);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(193.447);// -Math.toRadians(-12.744140625d);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 15;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(149.0625);// -Math.toRadians(30.673828125d);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 18;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(59.765);// -Math.toRadians(119.00390625d);

}
