package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    private final double deadzone = 0.08;

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

            HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("Stop", new InstantCommand(() -> drivetrain.stop()));
            eventMap.put("Stop Print", new InstantCommand(drivetrain::stop, drivetrain));

            // creates a trajectory using pathplanner
            makeTrajectory("test-path", DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                    eventMap);

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
    public void makeTrajectory(String name, double maxVelocity, double maxAcceleration,
            HashMap<String, Command> eventMap) throws IOException {
        ArrayList<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(name, maxVelocity, maxVelocity);

        eventMap.put("Set Inital Pose", new InstantCommand(() -> drivetrain
                .setInitialPose(trajectory.get(0).getInitialPose(), trajectory.get(0).getInitialPose().getRotation())));
        eventMap.put("Set Inital Pose Print", new PrintCommand("Set Initial Pose"));

        eventMap.put("End Path Print", new PrintCommand("End Path"));

        // PID controllers
        PIDConstants xConstants = new PIDConstants(Gains.kP, Gains.kI, Gains.kD);
        PIDConstants thetaConstants = new PIDConstants(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose,
                drivetrain::resetOdometry,
                xConstants,
                thetaConstants,
                drivetrain::setChassisSpeeds,
                eventMap,
                drivetrain);

        // adds the command to the chooser
        chooser.addOption(name, autoBuilder.fullAuto(trajectory));
    }
}