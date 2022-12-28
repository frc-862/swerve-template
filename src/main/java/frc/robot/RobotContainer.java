package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.IOException;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lightningUtil.logging.DataLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.ThetaGains;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    private final XboxController driver = new XboxController(0);
    private final double deadzone = 0.05;

    frc.robot.PPSwerveControllerCommand swerveCommand;

    // Creates our sendable chooser and Atuonomous dashboard tab
    private SendableChooser<Command> chooser = new SendableChooser<>();
    private ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -MathUtil.applyDeadband(driver.getLeftX(), deadzone),
                        () -> MathUtil.applyDeadband(driver.getLeftY(), deadzone),
                        () -> -MathUtil.applyDeadband(driver.getRightX(), deadzone)));

        ConfigureAutonomousCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Back button to reset feild centeric driving to current heading of the robot
        new Button(driver::getBackButton)
                .whenPressed(drivetrain::zeroYaw);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    private void ConfigureAutonomousCommands() {
        // Create a dashboard chooser for selecting autonomus command
        chooser.setDefaultOption("no path",
                new InstantCommand(() -> System.out.println("you should be doing nothing right now")));
        try {            // Creates a trajectory using pathplanner generated wpilib json files
            makeTrajectory("meter", DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
            // makeTrajectory makeTrajectory("circle");
            // makeTrajectory makeTrajectory("funny-path");
            makeTrajectory("test-path", 0.1, 0.1);
        } catch (IOException e) {
            e.printStackTrace();
            System.err.println("Failed to Make Trajectory");
        }

        autonomousTab.add("chooser", chooser);
    }

    /**
     * This method is gooing to create a swerve trajectory using pathplanners
     * generated wpilib json files. Max veloxity, max acceleration, and reversed
     * should be set when creating the paths in pathplanner
     * 
     * @param name name of the path in the deploy/pathplanner/generatedJSON
     *             folder (no ".wpilib.json")
     * @throws IOException
     */
    public void makeTrajectory(String name, double maxVelocity, double maxAcceleration) throws IOException {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(name, maxVelocity, maxVelocity);

        // PID controllers
        PIDController xController = new PIDController(Gains.kP, Gains.kI, Gains.kD);
        PIDController yController = new PIDController(Gains.kP, Gains.kI, Gains.kD);
        ProfiledPIDController thetaController = new ProfiledPIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD,
                new TrapezoidProfile.Constraints(
                        Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        2 * Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Adds generated swerve path to chooser
        swerveCommand = new frc.robot.PPSwerveControllerCommand(trajectory,
                drivetrain::getPose,
                drivetrain.getDriveKinematics(),
                xController,
                yController,
                thetaController,
                drivetrain::setStates,
                drivetrain);

        chooser.addOption(name,
                new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setInitialPose(trajectory.getInitialPose(),
                                trajectory.getInitialState().holonomicRotation)),
                        swerveCommand));

        DataLogger.addDataElement("desired holonomic roatation",
                () -> swerveCommand.getDesiredState().holonomicRotation.getDegrees());
    }
}