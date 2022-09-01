package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.IOException;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
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
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, () -> -MathUtil.applyDeadband(driver.getLeftX(), deadzone), () -> MathUtil.applyDeadband(driver.getLeftY(), deadzone), () -> -MathUtil.applyDeadband(driver.getRightX(), deadzone)));

        ConfigureAutonomousCommands();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Button(driver::getBackButton)
        .whenPressed(drivetrain::zeroGyroscope);
    }

    private void ConfigureAutonomousCommands() {
        chooser.setDefaultOption("no path", new InstantCommand(() -> System.out.println("you should be doing nothing right now")));
        makeTrajectory("1 meter", 0.1, 0.1, false);
        autonomousTab.add("chooser", chooser);
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public void makeTrajectory(String path, double maxVel, double maxAcel, boolean reversed) {
        Trajectory trajectory = PathPlanner.loadPath(path, maxVel, maxAcel, reversed);

        PIDController xController = new PIDController(0.00057741, 0, 0); //TODO tune me using sysid and configuring it by locking the wheels.
        PIDController yController = new PIDController(0.00057741, 0, 0); //TODO tune me using sysid 
        ProfiledPIDController thetaController = new ProfiledPIDController(, 0, 0, new TrapezoidProfile.Constraints(1, 1)); //TODO tune me 

        chooser.addOption(path, new SwerveControllerCommand(trajectory, 
            drivetrain::getPose, 
            drivetrain.getDriveKinematics(), 
            xController, 
            yController, 
            thetaController, 
            drivetrain::setStates, 
            drivetrain));
    }

}