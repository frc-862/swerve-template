package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.DrivetrainConstants.Gains;
import frc.robot.Constants.DrivetrainConstants.ThetaGains;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    private final XboxController driver = new XboxController(0);

    private final double deadzone = 0.1;

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
        new Button(driver::getBackButton)
                .whenPressed(drivetrain::zeroGyroscope);
    }

    private void ConfigureAutonomousCommands() {
        chooser.setDefaultOption("no path",
                new InstantCommand(() -> System.out.println("you should be doing nothing right now")));
        try {
            makeTrajectory("meter", 0.1, 0.1, false);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        autonomousTab.add("chooser", chooser);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public void makeTrajectory(String path, double maxVel, double maxAcel, boolean reversed) throws IOException {
        // PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, maxVel, maxAcel, reversed);
        Path p = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/unbrella.wpilib.json");
        Trajectory manual = TrajectoryUtil.fromPathweaverJson(p);
        // TrajectoryGenerator.generateTrajectory(
        // List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new
        // Rotation2d(0))),
        // new TrajectoryConfig(1, 1));

        PIDController xController = new PIDController(Gains.kP, Gains.kI, Gains.kD);
        PIDController yController = new PIDController(Gains.kP, Gains.kI, Gains.kD);
        ProfiledPIDController thetaController = new ProfiledPIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD,
                new TrapezoidProfile.Constraints(
                        Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        2 * Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // chooser.addOption(path, new PPSwerveControllerCommand(trajectory,
        //         drivetrain::getPose, drivetrain.getDriveKinematics(),
        //         xController,
        //         yController,
        //         thetaController,
        //         drivetrain::setStates,
        //         drivetrain));

        chooser.addOption("Path weaver", new SwerveControllerCommand(manual,
                drivetrain::getPose, drivetrain.getDriveKinematics(),
                xController,
                yController,
                thetaController,
                drivetrain::setStates,
                drivetrain));

        // drivetrain.setInitialPose(new Pose2d(new Translation2d(1, 3), new
        // Rotation2d()), new Rotation2d());

    }

}