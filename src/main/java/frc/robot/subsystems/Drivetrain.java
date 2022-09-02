package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.DrivetrainConstants.MAX_VOLTAGE;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.Gains;

public class Drivetrain extends SubsystemBase {

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
            
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);   

    private SwerveModuleState[] m_states;

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d());

    public Drivetrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);
    }

    public void zeroGyroscope() {
        navX.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - MathUtil.inputModulus(navX.getYaw() + 90, 0, 360));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, new Rotation2d());
    }

    public SwerveDriveKinematics getDriveKinematics() {
        return m_kinematics;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        if (m_states != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0
				&& chassisSpeeds.omegaRadiansPerSecond == 0) {
			m_states[0].speedMetersPerSecond = 0;
			m_states[1].speedMetersPerSecond = 0;
			m_states[2].speedMetersPerSecond = 0;
			m_states[3].speedMetersPerSecond = 0;
		} else {
			m_states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
		}
		setStates(m_states);
    }

    public void setStates(SwerveModuleState[] states) {
        if (states != null) {
			SwerveModuleState frontLeftState = states[0];
			SwerveModuleState frontRightState = states[1];
			SwerveModuleState backLeftState = states[2];
			SwerveModuleState backRightState = states[3];

			SwerveDriveKinematics.desaturateWheelSpeeds(states,
					MAX_VELOCITY_METERS_PER_SECOND);

			m_frontLeftModule.set(velocityToDriveVolts(frontLeftState.speedMetersPerSecond),
					frontLeftState.angle.getRadians());
			m_frontRightModule.set(velocityToDriveVolts(frontRightState.speedMetersPerSecond),
					frontRightState.angle.getRadians());
			m_backLeftModule.set(velocityToDriveVolts(backLeftState.speedMetersPerSecond),
					backLeftState.angle.getRadians());
			m_backRightModule.set(velocityToDriveVolts(backRightState.speedMetersPerSecond),
					backRightState.angle.getRadians());
		}
    }

    private double velocityToDriveVolts(double speedMetersPerSecond) {
		double ff = m_feedForward.calculate(speedMetersPerSecond);
		return MathUtil.clamp(ff, -MAX_VOLTAGE, MAX_VOLTAGE);
	}
    
    @Override
    public void periodic() {
        odometry.update(getGyroscopeRotation(), m_states);

        SmartDashboard.putNumber("fl", m_frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("bl", m_backLeftModule.getSteerAngle());
        SmartDashboard.putNumber("fr", m_frontRightModule.getSteerAngle());
        SmartDashboard.putNumber("br", m_backRightModule.getSteerAngle());

        SmartDashboard.putNumber("yaw", MathUtil.inputModulus(navX.getYaw(), 0, 360));
        SmartDashboard.putString("pose", odometry.getPoseMeters().toString());
    }
}