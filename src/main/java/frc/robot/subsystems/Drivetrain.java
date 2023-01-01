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
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lightningUtil.logging.DataLogger;
import frc.lightningUtil.swervelib.Mk3SwerveModuleHelper;
import frc.lightningUtil.swervelib.SwerveModule;
import frc.robot.Constants.DrivetrainConstants.Gains;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Drivetrain extends SubsystemBase {

    // creates our swerve kinematics using the robots track width and wheel base
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // creating new pigeon2 gyro
    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(1, "Canivore");

    // creating new pose, odometry, and cahssis speeds
    private Pose2d m_pose = new Pose2d();
    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), m_pose);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // creating our feed forward
    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);

    // field2d for displaying on the dashboard
    private final Field2d field2d = new Field2d();

    // creating our list of module states
    private SwerveModuleState[] m_states;

    // creating our modules
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    public Drivetrain() {
        // creates our drivetrain shuffleboard tab for displaying module data
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // put our field2d on the dashboard
        SmartDashboard.putData("Field", field2d);

        // making front left module
        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        // making front right module
        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        // making backleft module
        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        // making back right module
        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        // zero our gyro
        zeroYaw();

        // start logging data
        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);

    }

    @Override
    public void periodic() {
        // update our odometry and field2d
        updateOdomtery();
        field2d.setRobotPose(m_pose);
    }

    /**
     * This takes chassis speeds and converts them to module states and then sets
     * states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        if (m_states != null && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0
                && chassisSpeeds.omegaRadiansPerSecond == 0) {
            m_states[0] = new SwerveModuleState(0, new Rotation2d(-45));
            m_states[1] = new SwerveModuleState(0, new Rotation2d(45));
            m_states[2] = new SwerveModuleState(0, new Rotation2d(45));
            m_states[3] = new SwerveModuleState(0, new Rotation2d(-45));

            
        } else {
            m_states = kinematics.toSwerveModuleStates(chassisSpeeds);
        }
        setStates(m_states);
    }

    /**
     * This takes a list of module states and sets them to the modules.
     * 
     * @param states the list of module states to set
     */
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

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdomtery() {
        m_pose = m_odometry.update(getYaw2d(),
                stateFromModule(m_frontLeftModule), stateFromModule(m_frontRightModule),
                stateFromModule(m_backLeftModule), stateFromModule(m_backRightModule));
    }

    /**
     * Method to start logging data.
     */
    public void initLogging() {
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
    }

    /**
     * Method to set states of modules.
     */
    public void setStates(SwerveModuleState[] newStates) {
        m_states = newStates;
        updateOdomtery();
        updateDriveStates(m_states);

    }

    /**
     * Sets initial pose of robot in meters.
     * 
     * @param initalPosition the initial position of the robot
     * @param initalRotation the initial rotation(heading) of the robot
     */
    public void setInitialPose(Pose2d initalPosition, Rotation2d initalRotation) {
        pigeon.setYaw(initalRotation.getDegrees());
        m_pose = new Pose2d(initalPosition.getTranslation(), initalRotation);
        m_odometry = new SwerveDriveOdometry(kinematics,
                getYaw2d(), m_pose);

    }

    /**
     * Converts a velocity in meters per second to a voltage for the drive motors
     * using feedforward.
     * 
     * @param speedMetersPerSecond the velocity to convert
     * 
     * @return the clamped voltage to apply to the drive motors
     */
    private double velocityToDriveVolts(double speedMetersPerSecond) {
        double ff = m_feedForward.calculate(speedMetersPerSecond);
        return MathUtil.clamp(ff, -MAX_VOLTAGE, MAX_VOLTAGE);
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return the current pose of the robot in meters
     */
    public Rotation2d getYaw2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getYaw() - 90, 0, 360));
    }

    /**
     * Gets current state of module.
     * 
     * @return the current state of the specified module
     */
    public SwerveModuleState stateFromModule(SwerveModule swerveModule) {
        return new SwerveModuleState(swerveModule.getDriveVelocity(), new Rotation2d(swerveModule.getSteerAngle()));
    }

    /**
     * Converts percent output of joystick to a velocity in meters per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return the velocity in meters per second
     */
    public double percentOutputToMetersPerSecond(double percentOutput) {
        return percentOutput * MAX_VELOCITY_METERS_PER_SECOND;
    }

    /**
     * Converts percent output of joystick to a rotational velocity in omega radians
     * per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return
     */
    public double percentOutputToRadiansPerSecond(double percentOutput) {
        return percentOutput * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    /**
     * Zeroes the yaw of the pigeon.
     */
    public void zeroYaw() {
        pigeon.setYaw(0);
    }

    /**
     * Gets the current pose of the robot.
     */
    public Pose2d getPose() {
        return m_pose;
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose the pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, new Rotation2d());
    }

    /**
     * Gets the kinematics of the robot.
     * 
     * @return the kinematics of the robot
     */
    public SwerveDriveKinematics getDriveKinematics() {
        return kinematics;
    }

    /**
     * Gets the states of the modules.
     * 
     * @return the states of the modules
     */
    public SwerveModuleState[] getStates() {
        return m_states;
    }

    /**
     * Gets the front left module.
     * 
     * @return the front left module
     */
    public SwerveModule getM_frontLeftModule() {
        return m_frontLeftModule;
    }

    /**
     * Gets the front right module.
     * 
     * @return the front right module
     */
    public SwerveModule getM_frontRightModule() {
        return m_frontRightModule;
    }

    /**
     * Gets the back left module.
     * 
     * @return the back left module
     */
    public SwerveModule getM_backLeftModule() {
        return m_backLeftModule;
    }

    /**
     * Gets the back right module.
     * 
     * @return the back right module
     */
    public SwerveModule getM_backRightModule() {
        return m_backRightModule;
    }

    /**
     * Gets the chassis speeds.
     * 
     * @return the chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    /**
     * Sets the chassis speeds.
     * 
     * @param chassisSpeeds the chassis speeds to set
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
}