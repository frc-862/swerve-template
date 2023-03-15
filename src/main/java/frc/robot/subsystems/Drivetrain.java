package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.thunder.logging.DataLogger;
import frc.thunder.swervelib.Mk3ModuleConfiguration;
import frc.thunder.swervelib.Mk3SwerveModuleHelper;
import frc.thunder.swervelib.ModuleConfiguration;
import frc.thunder.swervelib.SwerveModule;

public class Drivetrain extends SubsystemBase {

    // Creates our swerve kinematics using the robots track width and wheel base
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // Creating new pigeon2 gyro
    private final WPI_Pigeon2 pigeon =
            new WPI_Pigeon2(DrivetrainConstants.PIGEON, DrivetrainConstants.PIGEON_CANBUS);

    // Creating new pose, odometry, and cahssis speeds
    private Pose2d pose = new Pose2d();
    private SwerveModulePosition[] modulePositions = {new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private SwerveDriveOdometry odometry =
            new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions, pose);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    // Creating our feed forward
    private final SimpleMotorFeedforward feedForward =
            new SimpleMotorFeedforward(Gains.kS, Gains.kV, Gains.kA);

    // Field2d for displaying on the dashboard
    private final Field2d field2d = new Field2d();

    // Creating our list of module states
    private SwerveModuleState[] states;

    // Creating our modules
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public Drivetrain() {
        // Creates our drivetrain shuffleboard tab for displaying module data
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Put our field2d on the dashboard
        SmartDashboard.putData("Field", field2d);

        Mk3ModuleConfiguration moduleConfiguration = new Mk3ModuleConfiguration();

        // Making front left module
        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0),
                moduleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST,
                DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

        // Making front right module
        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(2, 0),
                moduleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST,
                DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        // Making backleft module
        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(
                        4, 0),
                moduleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST,
                DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);

        // Making back right module
        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(6, 0),
                moduleConfiguration, Mk3SwerveModuleHelper.GearRatio.FAST,
                DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

        modulePositions[0] = frontLeftModule.getPosition();
        modulePositions[1] = frontRightModule.getPosition();
        modulePositions[2] = backLeftModule.getPosition();
        modulePositions[3] = backRightModule.getPosition();

        // Zero our gyro
        zeroYaw();

        // Start logging data
        initLogging();

        CommandScheduler.getInstance().registerSubsystem(this);

    }

    @Override
    public void periodic() {
        // Update our module positions, odometery, and field2d
        updateModulePositions();
        updateOdomtery();
        field2d.setRobotPose(pose);
        SmartDashboard.putString("pose", pose.getTranslation().toString());
    }

    /**
     * This takes chassis speeds and converts them to module states and then sets states.
     * 
     * @param chassisSpeeds the chassis speeds to convert to module states
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        if (states != null && chassisSpeeds.vxMetersPerSecond == 0
                && chassisSpeeds.vyMetersPerSecond == 0
                && chassisSpeeds.omegaRadiansPerSecond == 0) {
            states[0] = new SwerveModuleState(0,
                    new Rotation2d(DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE));
            states[1] = new SwerveModuleState(0,
                    new Rotation2d(DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE));
            states[2] = new SwerveModuleState(0,
                    new Rotation2d(DrivetrainConstants.BACK_LEFT_RESTING_ANGLE));
            states[3] = new SwerveModuleState(0,
                    new Rotation2d(DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE));

        } else {
            states = kinematics.toSwerveModuleStates(chassisSpeeds);
        }
        setStates(states);
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
                    DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

            frontLeftModule.set(velocityToDriveVolts(frontLeftState.speedMetersPerSecond),
                    frontLeftState.angle.getRadians());
            frontRightModule.set(velocityToDriveVolts(frontRightState.speedMetersPerSecond),
                    frontRightState.angle.getRadians());
            backLeftModule.set(velocityToDriveVolts(backLeftState.speedMetersPerSecond),
                    backLeftState.angle.getRadians());
            backRightModule.set(velocityToDriveVolts(backRightState.speedMetersPerSecond),
                    backRightState.angle.getRadians());
        }
    }

    /**
     * Updates odometry using the current yaw and module states.
     */
    public void updateOdomtery() {
        pose = odometry.update(getYaw2d(), modulePositions);
    }

    public void updateModulePositions() {
        modulePositions[0] = frontLeftModule.getPosition();
        modulePositions[1] = frontRightModule.getPosition();
        modulePositions[2] = backLeftModule.getPosition();
        modulePositions[3] = backRightModule.getPosition();
    }

    /**
     * Method to start logging data.
     */
    public void initLogging() {
        DataLogger.addDataElement("fl steer angle",
                () -> Math.toDegrees(frontLeftModule.getSteerAngle()));
        DataLogger.addDataElement("fl drive velocity", () -> frontLeftModule.getDriveVelocity());
        DataLogger.addDataElement("fr steer angle",
                () -> Math.toDegrees(frontRightModule.getSteerAngle()));
        DataLogger.addDataElement("fr drive velocity", () -> frontRightModule.getDriveVelocity());
        DataLogger.addDataElement("bl steer angle",
                () -> Math.toDegrees(backLeftModule.getSteerAngle()));
        DataLogger.addDataElement("bl drive velocity", () -> backLeftModule.getDriveVelocity());
        DataLogger.addDataElement("br steer angle",
                () -> Math.toDegrees(backRightModule.getSteerAngle()));
        DataLogger.addDataElement("br drive velocity", () -> backRightModule.getDriveVelocity());

        DataLogger.addDataElement("Heading", () -> getYaw2d().getDegrees());

        DataLogger.addDataElement("poseX", () -> getPose().getX());
        DataLogger.addDataElement("poseY", () -> getPose().getY());
    }

    /**
     * Method to set states of modules.
     */
    public void setStates(SwerveModuleState[] newStates) {
        states = newStates;
        updateModulePositions();
        updateOdomtery();
        updateDriveStates(states);

    }

    /**
     * Sets initial pose of robot in meters.
     * 
     * @param initalPosition the initial position of the robot
     * @param initalRotation the initial rotation(heading) of the robot
     */
    public void setInitialPose(Pose2d initalPosition, Rotation2d initalRotation) {
        pigeon.setYaw(initalRotation.getDegrees());
        pose = new Pose2d(initalPosition.getTranslation(), initalRotation);
        odometry = new SwerveDriveOdometry(kinematics, getYaw2d(), modulePositions, pose);

    }

    /**
     * Converts a velocity in meters per second to a voltage for the drive motors using feedforward.
     * 
     * @param speedMetersPerSecond the velocity to convert
     * 
     * @return the clamped voltage to apply to the drive motors
     */
    private double velocityToDriveVolts(double speedMetersPerSecond) {
        double ff = feedForward.calculate(speedMetersPerSecond);
        return MathUtil.clamp(ff, -DrivetrainConstants.MAX_VOLTAGE,
                DrivetrainConstants.MAX_VOLTAGE);
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
        return new SwerveModuleState(swerveModule.getDriveVelocity(),
                new Rotation2d(swerveModule.getSteerAngle()));
    }

    /**
     * Converts percent output of joystick to a velocity in meters per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return the velocity in meters per second
     */
    public double percentOutputToMetersPerSecond(double percentOutput) {
        return percentOutput * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    }

    /**
     * Converts percent output of joystick to a rotational velocity in omega radians per second.
     * 
     * @param percentOutput the percent output of the joystick
     * 
     * @return
     */
    public double percentOutputToRadiansPerSecond(double percentOutput) {
        return percentOutput * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
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
        return pose;
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose the pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw2d(), modulePositions, pose);
    }

    public void resyncEncoder() {
        frontLeftModule.setEncoderAngle();
        frontRightModule.setEncoderAngle();
        backLeftModule.setEncoderAngle();
        backRightModule.setEncoderAngle();
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
        return states;
    }

    /**
     * Gets the front left module.
     * 
     * @return the front left module
     */
    public SwerveModule getFrontLeftModule() {
        return frontLeftModule;
    }

    /**
     * Gets the front right module.
     * 
     * @return the front right module
     */
    public SwerveModule getFrontRightModule() {
        return frontRightModule;
    }

    /**
     * Gets the back left module.
     * 
     * @return the back left module
     */
    public SwerveModule getBackLeftModule() {
        return backLeftModule;
    }

    /**
     * Gets the back right module.
     * 
     * @return the back right module
     */
    public SwerveModule getBackRightModule() {
        return backRightModule;
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

    public void stop() {
        frontLeftModule.set(0, DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE);
        frontRightModule.set(0, DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE);
        backLeftModule.set(0, DrivetrainConstants.BACK_LEFT_RESTING_ANGLE);
        backRightModule.set(0, DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE);
    }
}
