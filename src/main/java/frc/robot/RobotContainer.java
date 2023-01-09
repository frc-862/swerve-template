package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lightningUtil.LightningContainer;
import frc.lightningUtil.auto.AutonomousCommandFactory;
import frc.lightningUtil.filter.JoystickFilter;
import frc.lightningUtil.filter.JoystickFilter.Mode;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends LightningContainer {
    // Creates our drivetrain subsystem
    private static final Drivetrain drivetrain = new Drivetrain();

    // Creates our driver controller and deadzone
    private static final XboxController driver = new XboxController(0);
    private static final JoystickFilter joystickFilter = new JoystickFilter(XboxControllerConstants.DEADBAND,
            XboxControllerConstants.MIN_POWER, XboxControllerConstants.MAX_POWER, Mode.CUBED);

    private static HashMap<String, Command> testPathMap = new HashMap<>();

    // Configure the button bindings
    @Override
    protected void configureButtonBindings() {
        // Back button to reset feild centeric driving to current heading of the robot
        new Trigger(driver::getBackButton)
                .onTrue(new InstantCommand(drivetrain::zeroYaw, drivetrain));
    }

    // Creates the autonomous commands
    @Override
    protected void configureAutonomousCommands() {
        // Creates a trajectory using pathplanner
        testPathMap.put("Starting Pose Print", new InstantCommand(() -> System.out.println(drivetrain.getPose())));
        testPathMap.put("Stop", new InstantCommand(drivetrain::stop));
        testPathMap.put("Stop Print",
                new InstantCommand(() -> System.out.println("Stopped at: " + drivetrain.getPose())));
        testPathMap.put("End Print",
                new InstantCommand(() -> System.out.println("Ended Path at: " + drivetrain.getPose())));
        AutonomousCommandFactory.makeTrajectory("test-path", DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, DrivetrainConstants.DRIVE_PID_CONSTANTS,
                DrivetrainConstants.THETA_PID_CONSTANTS, testPathMap, drivetrain);
    }

    @Override
    protected void configureDefaultCommands() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(
                new SwerveDrive(drivetrain, () -> -joystickFilter.filter(driver.getLeftX()),
                        () -> joystickFilter.filter(driver.getLeftY()),
                        () -> -joystickFilter.filter(driver.getRightX())));

    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void releaseDefaultCommands() {
    }

    @Override
    protected void initializeDashboardCommands() {
    }

    @Override
    protected void configureFaultCodes() {
    }

    @Override
    protected void configureFaultMonitors() {
    }
}