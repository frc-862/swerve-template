package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;

// import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.thunder.LightningContainer;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.filter.JoystickFilter;
import frc.thunder.filter.JoystickFilter.Mode;

public class RobotContainer extends LightningContainer {
        // Creates our drivetrain subsystem
        private static final Drivetrain drivetrain = new Drivetrain();

        // Creates vision
        private static final Vision vision = new Vision();

        // Creates our driver controller and deadzone
        private static final XboxController driver = new XboxController(0);
        private static final JoystickFilter joystickFilter = new JoystickFilter(
                        XboxControllerConstants.DEADBAND, XboxControllerConstants.MIN_POWER,
                        XboxControllerConstants.MAX_POWER, Mode.CUBED);

        private static HashMap<String, Command> testPathMap = new HashMap<>();

        // private static AutonomousCommandFactory autoFactory = new
        // AutonomousCommandFactory(drivetrain::getPose,
        // drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
        // DrivetrainConstants.DRIVE_PID_CONSTANTS, DrivetrainConstants.DRIVE_PID_CONSTANTS,
        // drivetrain::setStates, drivetrain);

        // Configure the button bindings
        @Override
        protected void configureButtonBindings() {
                // Back button to reset feild centeric driving to current heading of the robot
                new Trigger(driver::getBackButton)
                                .onTrue(new InstantCommand(drivetrain::zeroYaw, drivetrain));
                new Trigger(driver::getAButton).whileTrue(new AutoAlign(drivetrain, vision));
        }

        // Creates the autonomous commands
        @Override
        protected void configureAutonomousCommands() {
                // Creates a trajectory using pathplanner
                // testPathMap.put("Starting Pose Print",
                // new InstantCommand(() -> System.out.println(drivetrain.getPose())));
                // testPathMap.put("Stop", new InstantCommand(drivetrain::stop));
                // testPathMap.put("Stop Print",
                // new InstantCommand(() -> System.out.println("Stopped at: " +
                // drivetrain.getPose())));
                // testPathMap.put("End Print",
                // new InstantCommand(() -> System.out.println("Ended Path at: " +
                // drivetrain.getPose())));
                // autoFactory.makeTrajectory("test-path", testPathMap, new PathConstraints(3, 3),
                // new PathConstraints(3, 3));
        }

        @Override
        protected void configureDefaultCommands() {
                // Set up the default command for the drivetrain.
                // The controls are for field-oriented driving:
                // Left stick Y axis -> forward and backwards movement
                // Left stick X axis -> left and right movement
                // Right stick X axis -> rotation
                drivetrain.setDefaultCommand(new SwerveDrive(drivetrain,
                                () -> -joystickFilter.filter(driver.getLeftX()),
                                () -> joystickFilter.filter(driver.getLeftY()),
                                () -> -joystickFilter.filter(driver.getRightX())));

        }

        @Override
        protected void configureSystemTests() {}

        @Override
        protected void releaseDefaultCommands() {}

        @Override
        protected void initializeDashboardCommands() {}

        @Override
        protected void configureFaultCodes() {}

        @Override
        protected void configureFaultMonitors() {}
}
