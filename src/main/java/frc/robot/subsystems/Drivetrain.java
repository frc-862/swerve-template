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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lightningUtil.logging.DataLogger;
import frc.lightningUtil.swervelib.Mk3SwerveModuleHelper;
import frc.lightningUtil.swervelib.Mk4SwerveModuleHelper;
import frc.lightningUtil.swervelib.SwerveModule;
import frc.robot.Constants.DrivetrainConstants.Gains;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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

    // Creating new navX gyro
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(1, "Canivore");
    private final AHRS navX = new AHRS(Port.kMXP);

    // Creating our pose and odometry
    private Pose2d m_pose = new Pose2d();
    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getYaw2d(), m_pose);

    // Creating our feed forward
    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);

    // Field2d for displaying on the dashboard
    private final Field2d m_field2d = new Field2d();

    // Creating our list of module states
    private SwerveModuleState[] m_states;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

    // Creating our modules
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    public Drivetrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Making front left module
        m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                    Mk4SwerveModuleHelper.GearRatio.L3,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        // Making front right module
        m_frontRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                    Mk4SwerveModuleHelper.GearRatio.L3,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        // Making backleft module
        m_backLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                    Mk4SwerveModuleHelper.GearRatio.L3,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        // Making back right module
        m_backRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                    Mk4SwerveModuleHelper.GearRatio.L3,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        // Zero our gyro
        setInitialPose(new Pose2d(), getYaw2d());
        SmartDashboard.putData("Field", m_field2d);
        initLogging();
        CommandScheduler.getInstance().registerSubsystem(this);

    }

    @Override
    public void periodic() {
        updateOdomtery();
        m_field2d.setRobotPose(m_pose);

        SmartDashboard.putNumber("fl", Math.toDegrees(m_frontLeftModule.getSteerAngle()));
        SmartDashboard.putNumber("bl", Math.toDegrees(m_backLeftModule.getSteerAngle()));
        SmartDashboard.putNumber("fr", Math.toDegrees(m_frontRightModule.getSteerAngle()));
        SmartDashboard.putNumber("br", Math.toDegrees(m_backRightModule.getSteerAngle()));

        SmartDashboard.putNumber("pigeon yaw", pigeon.getYaw());
        SmartDashboard.putNumber("navX yaw",  navX.getYaw());
        SmartDashboard.putNumber("pose yaw", getPose().getRotation().getDegrees());

    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
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

    public void updateDriveStates(SwerveModuleState[] states) {
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

    public void updateOdomtery() {
        m_pose = m_odometry.update(getYaw2d(),
                stateFromModule(m_frontLeftModule), stateFromModule(m_frontRightModule),
                stateFromModule(m_backLeftModule), stateFromModule(m_backRightModule));
    }

    public void initLogging() {
        System.out.println("********************** initlogging drivetrain");
        DataLogger.addDataElement("fl steer angle", () -> Math.toDegrees(m_frontLeftModule.getSteerAngle()));
        DataLogger.addDataElement("fl drive velocity", () -> m_frontLeftModule.getDriveVelocity());
        DataLogger.addDataElement("fr steer angle", () -> Math.toDegrees(m_frontRightModule.getSteerAngle()));
        DataLogger.addDataElement("fr drive velocity", () -> m_frontRightModule.getDriveVelocity());
        DataLogger.addDataElement("bl steer angle", () -> Math.toDegrees(m_backLeftModule.getSteerAngle()));
        DataLogger.addDataElement("bl drive velocity", () -> m_backLeftModule.getDriveVelocity());
        DataLogger.addDataElement("br steer angle", () -> Math.toDegrees(m_backRightModule.getSteerAngle()));
        DataLogger.addDataElement("br drive velocity", () -> m_backRightModule.getDriveVelocity());

        DataLogger.addDataElement("Heading", () -> getYaw2d().getDegrees());

        DataLogger.addDataElement("poseX", () -> getPose().getX());
        DataLogger.addDataElement("poseY", () -> getPose().getY());
        DataLogger.addDataElement("pose rotation", () -> getPose().getRotation().getDegrees());
    }

    public void setStates(SwerveModuleState[] newStates) {
        m_states = newStates;
        updateOdomtery();
        updateDriveStates(m_states);

    }

    public void setInitialPose(Pose2d initalPosition, Rotation2d initalRotation) {
        pigeon.setYaw(initalRotation.getDegrees());
        m_pose = new Pose2d(initalPosition.getTranslation(), initalRotation);
        m_odometry = new SwerveDriveOdometry(m_kinematics,
                getYaw2d(), m_pose);

    }

    private double velocityToDriveVolts(double speedMetersPerSecond) {
        double ff = m_feedForward.calculate(speedMetersPerSecond);
        return MathUtil.clamp(ff, -MAX_VOLTAGE, MAX_VOLTAGE);
    }

    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));// Rotation2d.fromDegrees(360 - MathUtil.inputModulus(pigeon.getYaw() + 90, 0, 360));
    }

    private SwerveModuleState stateFromModule(SwerveModule swerveModule) {
        return new SwerveModuleState(swerveModule.getDriveVelocity(), new Rotation2d(swerveModule.getSteerAngle()));
    }

    public void zeroYaw() {
        pigeon.setYaw(0);
        navX.zeroYaw();
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, new Rotation2d());
    }

    public SwerveDriveKinematics getDriveKinematics() {
        return m_kinematics;
    }

    public SwerveModuleState[] getStates() {
        return m_states;
    }

    public SwerveModule getM_frontLeftModule() {
        return m_frontLeftModule;
    }

    public SwerveModule getM_frontRightModule() {
        return m_frontRightModule;
    }

    public SwerveModule getM_backLeftModule() {
        return m_backLeftModule;
    }

    public SwerveModule getM_backRightModule() {
        return m_backRightModule;
    }

    public ChassisSpeeds getM_chassisSpeeds() {
        return m_chassisSpeeds;
    }

    public void setM_chassisSpeeds(ChassisSpeeds m_chassisSpeeds) {
        this.m_chassisSpeeds = m_chassisSpeeds;
    }
}