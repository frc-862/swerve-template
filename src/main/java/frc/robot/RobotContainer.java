package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.IOException;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    // creates our drivetrain subsystem
    private final Drivetrain drivetrain = new Drivetrain();

    // creates our driver controller and deadzone
    private final XboxController driver = new XboxController(0);
    private final double deadzone = 0.15;

    // creates our sendable chooser and Atuonomous dashboard tab
    private SendableChooser<Command> chooser = new SendableChooser<>();
    private ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

    public RobotContainer() {
        // set up the default command for the drivetrain.
        // the controls are for field-oriented driving:
        // left stick Y axis -> forward and backwards movement
        // left stick X axis -> left and right movement
        // right stick X axis -> rotation
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -MathUtil.applyDeadband(driver.getLeftX(), deadzone),
                        () -> MathUtil.applyDeadband(driver.getLeftY(), deadzone),
                        () -> -MathUtil.applyDeadband(driver.getRightX(), deadzone)));

        ConfigureAutonomousCommands();
        configureButtonBindings();
    }

    // configure the button bindings
    private void configureButtonBindings() {
        // back button to reset feild centeric driving to current heading of the robot
        new Button(driver::getBackButton)
                .whenPressed(drivetrain::zeroYaw);
    }

    // returns the autonomous command
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    // creates the autonomous commands
    private void ConfigureAutonomousCommands() {
        // create a dashboard chooser for selecting autonomus command
        chooser.setDefaultOption("no path",
                new InstantCommand(() -> System.out.println("you should be doing nothing right now")));

        try {
            // creates a trajectory using pathplanner
            makeTrajectory("meter", DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

        } catch (IOException e) {
            e.printStackTrace();
            System.err.println("Failed to Make Trajectory");
        }

        // adds the chooser to the atuonomous tab
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
        PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);

        // enables continuous input for the theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // adds generated swerve path to chooser
        PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(trajectory, 
        drivetrain::getPose, 
        xController, 
        yController, 
        thetaController, 
        drivetrain::setChassisSpeeds, 
        drivetrain);

        // adds the command to the chooser
        chooser.addOption(name,
                // a sequential command group that will set the initial pose of the robot and
                // run the path
                new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.setInitialPose(trajectory.getInitialPose(),
                                trajectory.getInitialState().holonomicRotation)),
                        swerveCommand));
    }
}